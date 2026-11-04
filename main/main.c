#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_task_wdt.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_continuous.h"
#include "esp_heap_caps.h"
#include "driver/dac_oneshot.h"
#include "dfplayer_mini.h"
#include "led.h"
#include "esp_random.h"

// --- Broches ---
#define TX_GPIO           17 // ESP32 -> RX du DFPlayer
#define RX_GPIO           16 // TX du DFPlayer -> ESP32
#define DFPLAYER_BUSY_GPIO 23
#define POT_GPIO          34 // Potentiomètre : clé de l'inversion (ADC1_CH6)
#define AUDIO_IN_GPIO     35 // DAC_R du DFPlayer -> entrée à décoder (ADC1_CH7)
#define JOYSTICK_X_GPIO   32 // Joystick VRX : tonalité (ADC1_CH4)
#define JOYSTICK_Y_GPIO   33 // Joystick VRY : vitesse (ADC1_CH5)
#define AUDIO_OUT_DAC     DAC_CHAN_0 // GPIO25 -> 1k -> condensateur -> entrée R du PAM8403
#define LED_POT_GPIO      GPIO_NUM_0  // LED qui guide/verrouille sur le potentiomètre (clé)
#define LED_JOY_GPIO      GPIO_NUM_15 // LED qui guide/verrouille sur le joystick (tonalité+vitesse)

// --- Échantillonnage ---
// 44,1 kHz. L'entrée passe par l'ADC continu (DMA) : adc_oneshot_read prenait ~40 µs par lecture et
// plafonnait la boucle à ~21,9 kHz. La sortie reste en dac_oneshot cadencé par la boucle (sur l'ESP32,
// le DAC en DMA utilise le même I2S0 que l'ADC continu, impossible d'avoir les deux).
// On calcule l'échéance de chaque échantillon à partir du compteur (voir audio_task) pour que le taux
// réel soit exactement FS_HZ même si la période n'est pas un nombre entier de µs.
#define FS_HZ             44100

// --- ADC continu : les 4 entrées (audio, pot, joystick X/Y) tournent en boucle dans le motif DMA,
// donc chacune est échantillonnée à FS_HZ, à intervalles réguliers. ---
#define ADC_NUM_CHANNELS   4
#define ADC_FRAME_RESULTS  256 // 64 échantillons audio par trame (~1,45 ms)
#define ADC_FRAME_BYTES    (ADC_FRAME_RESULTS * SOC_ADC_DIGI_RESULT_BYTES)
#define ADC_POOL_BYTES     (16 * 1024) // ~46 ms de marge si la tâche audio est retardée
// File entre l'ADC et la boucle de sortie. Les horloges ADC et esp_timer dérivent un peu l'une par
// rapport à l'autre : on garde le niveau entre 0 et AUDIO_FIFO_HIGH, sinon on jette l'excédent.
#define AUDIO_FIFO_LEN     2048 // puissance de 2
#define AUDIO_FIFO_REFILL  128  // on relit l'ADC quand la file descend sous ce niveau
#define AUDIO_FIFO_HIGH    1024 // au-delà (~23 ms de latence), on revient à AUDIO_FIFO_REFILL

// --- Filtre passe-bande de l'inversion (comme le script Python : butter(8,[300,3000],bandpass),
// approximé ici par un passe-haut 300Hz 4e ordre + passe-bas 3000Hz 4e ordre en cascade = 8e ordre) ---
#define BANDPASS_HP_FC_HZ 300.0f
#define BANDPASS_LP_FC_HZ 3000.0f
#define BIQUAD_Q1         0.5412f
#define BIQUAD_Q2         1.3066f

// --- Inversion (potentiomètre = clé) : plage fixe, la cible exacte est tirée au hasard au boot ---
#define CARRIER_TABLE_SIZE 1024
#define CARRIER_TABLE_SHIFT 22 // 32 - log2(1024)
#define CARRIER_MIN_HZ    2800.0f
#define CARRIER_MAX_HZ    3800.0f
#define F_CLE_MARGIN_HZ   80.0f // marge gardée aux bords pour que la cible ne tombe pas trop près du bout
#define F_CLE_TOLERANCE_HZ 40.0f // élargi pour être plus clément
#define F_CLE_DEADZONE_HZ 60.0f // exclu autour de la position actuelle du potentiomètre au boot
#define F_CLE_FEEDBACK_RANGE_HZ ((CARRIER_MAX_HZ - CARRIER_MIN_HZ) / 2.0f) // échelle du clignotement (pas la tolérance)

// --- Tonalité (joystick X) : shelf RBJ, même fréquences que l'encodeur. Plage fixe, cible aléatoire. ---
#define SHELF_BAS_FC_HZ   600.0f
#define SHELF_HAUT_FC_HZ  1500.0f
#define TONALITE_DB_MIN   -20.0f
#define TONALITE_DB_MAX    20.0f
#define TONALITE_DB_MARGIN 6.0f
#define TONALITE_TOLERANCE_DB 4.0f // élargi pour être plus clément
#define TONALITE_DB_NEUTRE 0.0f // position de repos du joystick (centré) -> la cible ne doit pas tomber ici
#define TONALITE_DB_DEADZONE 10.0f
#define TONALITE_FEEDBACK_RANGE_DB ((TONALITE_DB_MAX - TONALITE_DB_MIN) / 2.0f) // échelle du clignotement

// --- Vitesse (joystick Y) : relecture à débit variable dans un tampon circulaire. Plage fixe, cible aléatoire. ---
// Backlog max avant que le tampon ne boucle : 65536 échantillons = ~1,5 s à 44,1 kHz (128 Ko).
// Découpé en 4 blocs de 32 Ko, car un seul bloc de 128 Ko ne tient pas dans le plus grand morceau
// libre de la RAM de l'ESP32. Longueur en puissance de 2 -> bouclage par masque.
#define SPEED_CHUNK_SHIFT   14
#define SPEED_CHUNK_LEN     (1 << SPEED_CHUNK_SHIFT)
#define SPEED_NUM_CHUNKS    4
#define SPEED_BUF_LEN       (SPEED_CHUNK_LEN * SPEED_NUM_CHUNKS)
#define SPEED_AT(chunks, i) ((chunks)[(i) >> SPEED_CHUNK_SHIFT][(i) & (SPEED_CHUNK_LEN - 1)])
#define PLAYBACK_RATE_MIN   0.5f
#define PLAYBACK_RATE_MAX   1.5f
#define PLAYBACK_RATE_MARGIN 0.1f
#define PLAYBACK_RATE_TOLERANCE 0.06f // élargi pour être plus clément
#define PLAYBACK_RATE_NEUTRE 1.0f // position de repos du joystick (centré) -> la cible ne doit pas tomber ici
#define PLAYBACK_RATE_DEADZONE 0.15f
#define PLAYBACK_RATE_FEEDBACK_RANGE ((PLAYBACK_RATE_MAX - PLAYBACK_RATE_MIN) / 2.0f) // échelle du clignotement

// --- Lissages / gains ---
#define CENTER_TRACK_ALPHA (8.0f / (float)FS_HZ) // constante de temps ~125 ms, quel que soit FS_HZ
#define CTRL_SMOOTH_ALPHA  0.05f
// Il faut rester dans la tolérance pendant ce nombre de lectures consécutives (~10ms chacune) avant
// que ça verrouille -> évite qu'un bruit ADC ou un passage transitoire d'une fraction de seconde
// verrouille par accident. ~0.8s ici.
#define OK_STREAK_REQUIRED 80
#define OUTPUT_GAIN        1.0f // Trop haut (ex: 3.0) sature le DAC en permanence -> bruit d'écrêtage
// Une fois l'énigme résolue (les 3 verrouillés), le volume monte de 30 %. Transition progressive
// (~50 ms) pour ne pas faire de "pop" au moment du déverrouillage.
#define SOLVED_VOLUME_BOOST 1.3f
#define GAIN_RAMP_ALPHA    (1.0f / (0.05f * (float)FS_HZ))

// --- LED indicatrice : clignote plus vite en se rapprochant, fixe quand c'est bon ---
#define LED_BLINK_FAST_MS  80
#define LED_BLINK_SLOW_MS  600

static const char *TAG = "[SPEAKER]";
static const char *TAG_AUDIO = "[AUDIO]";

typedef struct {
    float b0, b1, b2, a1, a2;
    float x1, x2, y1, y2;
} biquad_t;

static void biquad_init_lowpass(biquad_t *bq, float fc_hz, float q, float fs_hz)
{
    float w0 = 2.0f * (float)M_PI * fc_hz / fs_hz;
    float alpha = sinf(w0) / (2.0f * q);
    float cosw0 = cosf(w0);

    float b0 = (1.0f - cosw0) / 2.0f;
    float b1 = 1.0f - cosw0;
    float b2 = (1.0f - cosw0) / 2.0f;
    float a0 = 1.0f + alpha;
    float a1 = -2.0f * cosw0;
    float a2 = 1.0f - alpha;

    bq->b0 = b0 / a0;
    bq->b1 = b1 / a0;
    bq->b2 = b2 / a0;
    bq->a1 = a1 / a0;
    bq->a2 = a2 / a0;
    bq->x1 = bq->x2 = bq->y1 = bq->y2 = 0.0f;
}

static void biquad_init_highpass(biquad_t *bq, float fc_hz, float q, float fs_hz)
{
    float w0 = 2.0f * (float)M_PI * fc_hz / fs_hz;
    float alpha = sinf(w0) / (2.0f * q);
    float cosw0 = cosf(w0);

    float b0 = (1.0f + cosw0) / 2.0f;
    float b1 = -(1.0f + cosw0);
    float b2 = (1.0f + cosw0) / 2.0f;
    float a0 = 1.0f + alpha;
    float a1 = -2.0f * cosw0;
    float a2 = 1.0f - alpha;

    bq->b0 = b0 / a0;
    bq->b1 = b1 / a0;
    bq->b2 = b2 / a0;
    bq->a1 = a1 / a0;
    bq->a2 = a2 / a0;
    bq->x1 = bq->x2 = bq->y1 = bq->y2 = 0.0f;
}

// Shelf RBJ (pente S=1), mêmes formules que le script Python. type_bas=true -> "bas", false -> "haut".
static void biquad_set_shelf(biquad_t *bq, bool type_bas, float fc_hz, float gain_db, float fs_hz)
{
    float A = powf(10.0f, gain_db / 40.0f);
    float w0 = 2.0f * (float)M_PI * fc_hz / fs_hz;
    float c = cosf(w0);
    float s = sinf(w0);
    float alpha = (s / 2.0f) * sqrtf(2.0f); // pente S = 1
    float k = 2.0f * sqrtf(A) * alpha;

    float b0, b1, b2, a0, a1, a2;
    if (type_bas) {
        b0 = A * ((A + 1.0f) - (A - 1.0f) * c + k);
        b1 = 2.0f * A * ((A - 1.0f) - (A + 1.0f) * c);
        b2 = A * ((A + 1.0f) - (A - 1.0f) * c - k);
        a0 = (A + 1.0f) + (A - 1.0f) * c + k;
        a1 = -2.0f * ((A - 1.0f) + (A + 1.0f) * c);
        a2 = (A + 1.0f) + (A - 1.0f) * c - k;
    } else {
        b0 = A * ((A + 1.0f) + (A - 1.0f) * c + k);
        b1 = -2.0f * A * ((A - 1.0f) + (A + 1.0f) * c);
        b2 = A * ((A + 1.0f) + (A - 1.0f) * c - k);
        a0 = (A + 1.0f) - (A - 1.0f) * c + k;
        a1 = 2.0f * ((A - 1.0f) - (A + 1.0f) * c);
        a2 = (A + 1.0f) - (A - 1.0f) * c - k;
    }

    bq->b0 = b0 / a0;
    bq->b1 = b1 / a0;
    bq->b2 = b2 / a0;
    bq->a1 = a1 / a0;
    bq->a2 = a2 / a0;
    // Les états (x1,x2,y1,y2) ne sont pas remis à zéro : seuls les coefficients changent quand le
    // joystick bouge, pour éviter un clic audible à chaque mise à jour.
}

static inline float biquad_process(biquad_t *bq, float x)
{
    float y = bq->b0 * x + bq->b1 * bq->x1 + bq->b2 * bq->x2 - bq->a1 * bq->y1 - bq->a2 * bq->y2;
    bq->x2 = bq->x1;
    bq->x1 = x;
    bq->y2 = bq->y1;
    bq->y1 = y;
    return y;
}

static float carrier_table[CARRIER_TABLE_SIZE];

static void carrier_table_init(void)
{
    for (int i = 0; i < CARRIER_TABLE_SIZE; i++) {
        carrier_table[i] = cosf(2.0f * (float)M_PI * (float)i / (float)CARRIER_TABLE_SIZE);
    }
}

typedef struct {
    adc_oneshot_unit_handle_t adc_handle; // seulement pour la mesure au boot, libéré ensuite
    adc_continuous_handle_t adc_cont;
    adc_channel_t audio_channel;
    adc_channel_t pot_channel;
    adc_channel_t joy_x_channel;
    adc_channel_t joy_y_channel;
    dac_oneshot_handle_t dac_handle;
    led_t led_pot;
    led_t led_joy;
    float cible_carrier_hz;
    float cible_tonalite_db;
    float cible_playback_rate;
} audio_io_t;

static float random_float_range(float min, float max)
{
    float frac = (float)esp_random() / (float)UINT32_MAX; // 0..1
    return min + frac * (max - min);
}

// Comme random_float_range, mais exclut une zone autour de "centre" (± deadzone) : utile pour les
// contrôles qui reviennent tout seuls à une position neutre (joystick) -> sans ça, la cible aurait
// une chance non négligeable de tomber près de cette position neutre et de "se déverrouiller seule".
static float random_float_excluding_zone(float min, float max, float centre, float deadzone)
{
    float lo_lo = min, lo_hi = centre - deadzone;
    float hi_lo = centre + deadzone, hi_hi = max;
    float span_lo = (lo_hi > lo_lo) ? (lo_hi - lo_lo) : 0.0f;
    float span_hi = (hi_hi > hi_lo) ? (hi_hi - hi_lo) : 0.0f;
    float total = span_lo + span_hi;
    if (total <= 0.0f) {
        return random_float_range(min, max); // la zone interdite mange toute la plage, tant pis
    }
    float pick = random_float_range(0.0f, total);
    return (pick < span_lo) ? (lo_lo + pick) : (hi_lo + (pick - span_lo));
}

static float measure_channel_average(audio_io_t *io, adc_channel_t channel, int samples)
{
    int64_t sum = 0;
    for (int i = 0; i < samples; i++) {
        int raw = 0;
        adc_oneshot_read(io->adc_handle, channel, &raw);
        sum += raw;
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    return (float)sum / (float)samples;
}

static void audio_io_add_channel(audio_io_t *io, int gpio, adc_channel_t *out_channel)
{
    adc_unit_t unit;
    adc_oneshot_io_to_channel(gpio, &unit, out_channel);

    adc_oneshot_chan_cfg_t channel_config = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten = ADC_ATTEN_DB_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(io->adc_handle, *out_channel, &channel_config));
}

static void audio_io_init(audio_io_t *io)
{
    adc_unit_t unit_audio;
    adc_channel_t dummy;
    adc_oneshot_io_to_channel(AUDIO_IN_GPIO, &unit_audio, &dummy);

    // Toutes les entrées analogiques (audio, potentiomètre, joystick X/Y) doivent être sur la même
    // unité ADC1 : une seule instance adc_oneshot est créée pour les quatre.
    adc_oneshot_unit_init_cfg_t unit_config = {
        .unit_id = unit_audio,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_config, &io->adc_handle));

    audio_io_add_channel(io, AUDIO_IN_GPIO, &io->audio_channel);
    audio_io_add_channel(io, POT_GPIO, &io->pot_channel);
    audio_io_add_channel(io, JOYSTICK_X_GPIO, &io->joy_x_channel);
    audio_io_add_channel(io, JOYSTICK_Y_GPIO, &io->joy_y_channel);

    dac_oneshot_config_t dac_config = {
        .chan_id = AUDIO_OUT_DAC,
    };
    ESP_ERROR_CHECK(dac_oneshot_new_channel(&dac_config, &io->dac_handle));

    io->led_pot = led_init(LED_POT_GPIO);
    io->led_joy = led_init(LED_JOY_GPIO);

    // Peu importe la raison exacte (rappel au centre du joystick pas parfait, ou potentiomètre resté
    // sur une vieille position de test) : on mesure où sont RÉELLEMENT les 3 contrôles au boot, et on
    // exclut une zone autour de chacun. Sinon la cible aléatoire pourrait tomber pile là où un contrôle
    // se trouve déjà, et ça se "déverrouille" tout seul sans que le joueur touche à rien.
    float repos_pot_norm = measure_channel_average(io, io->pot_channel, 100) / 4095.0f;
    float repos_joy_x_norm = measure_channel_average(io, io->joy_x_channel, 100) / 4095.0f;
    float repos_joy_y_norm = measure_channel_average(io, io->joy_y_channel, 100) / 4095.0f;
    float repos_carrier_hz = CARRIER_MIN_HZ + repos_pot_norm * (CARRIER_MAX_HZ - CARRIER_MIN_HZ);
    float repos_tonalite_db = TONALITE_DB_MIN + repos_joy_x_norm * (TONALITE_DB_MAX - TONALITE_DB_MIN);
    float repos_playback_rate = PLAYBACK_RATE_MIN + repos_joy_y_norm * (PLAYBACK_RATE_MAX - PLAYBACK_RATE_MIN);
    ESP_LOGI(TAG_AUDIO, "Repos mesure: potentiometre -> %.1fHz   joystick X -> %.1fdB   joystick Y -> %.3fx",
             repos_carrier_hz, repos_tonalite_db, repos_playback_rate);

    io->cible_carrier_hz = random_float_excluding_zone(
        CARRIER_MIN_HZ + F_CLE_MARGIN_HZ, CARRIER_MAX_HZ - F_CLE_MARGIN_HZ,
        repos_carrier_hz, F_CLE_DEADZONE_HZ);
    io->cible_tonalite_db = random_float_excluding_zone(
        TONALITE_DB_MIN + TONALITE_DB_MARGIN, TONALITE_DB_MAX - TONALITE_DB_MARGIN,
        repos_tonalite_db, TONALITE_DB_DEADZONE);
    io->cible_playback_rate = random_float_excluding_zone(
        PLAYBACK_RATE_MIN + PLAYBACK_RATE_MARGIN, PLAYBACK_RATE_MAX - PLAYBACK_RATE_MARGIN,
        repos_playback_rate, PLAYBACK_RATE_DEADZONE);

    ESP_LOGI(TAG_AUDIO, "=== REPONSES (a ne pas montrer au joueur) ===");
    ESP_LOGI(TAG_AUDIO, "  potentiometre (porteuse) : %.1f Hz", io->cible_carrier_hz);
    ESP_LOGI(TAG_AUDIO, "  joystick X (tonalite)    : %.1f dB", io->cible_tonalite_db);
    ESP_LOGI(TAG_AUDIO, "  joystick Y (vitesse)     : %.3f x", io->cible_playback_rate);
    ESP_LOGI(TAG_AUDIO, "==============================================");

    // L'ADC1 ne peut pas être en oneshot et en continu à la fois : on libère le oneshot (qui n'a
    // servi qu'à la mesure de repos ci-dessus) et on passe tout en continu. Démarré ici, depuis le
    // cœur 0, pour que l'interruption DMA tourne sur le cœur 0 et ne perturbe pas la boucle audio.
    ESP_ERROR_CHECK(adc_oneshot_del_unit(io->adc_handle));
    io->adc_handle = NULL;

    adc_continuous_handle_cfg_t cont_cfg = {
        .max_store_buf_size = ADC_POOL_BYTES,
        .conv_frame_size = ADC_FRAME_BYTES,
        .flags.flush_pool = 1, // si le pool déborde, on jette le vieux plutôt que de bloquer
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&cont_cfg, &io->adc_cont));

    adc_digi_pattern_config_t pattern[ADC_NUM_CHANNELS];
    const adc_channel_t channels[ADC_NUM_CHANNELS] = {
        io->audio_channel, io->pot_channel, io->joy_x_channel, io->joy_y_channel,
    };
    for (int i = 0; i < ADC_NUM_CHANNELS; i++) {
        pattern[i].atten = ADC_ATTEN_DB_12;
        pattern[i].channel = channels[i];
        pattern[i].unit = ADC_UNIT_1;
        pattern[i].bit_width = ADC_BITWIDTH_12;
    }
    adc_continuous_config_t dig_cfg = {
        .pattern_num = ADC_NUM_CHANNELS,
        .adc_pattern = pattern,
        .sample_freq_hz = FS_HZ * ADC_NUM_CHANNELS,
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_continuous_config(io->adc_cont, &dig_cfg));
    ESP_ERROR_CHECK(adc_continuous_start(io->adc_cont));
    ESP_LOGI(TAG_AUDIO, "ADC continu demarre: %d voies x %d Hz", ADC_NUM_CHANNELS, FS_HZ);
}

// --- Entrée : trames DMA -> file audio + dernières valeurs des contrôles ---
typedef struct {
    uint8_t raw[ADC_FRAME_BYTES];
    adc_continuous_data_t parsed[ADC_FRAME_RESULTS];
    uint16_t fifo[AUDIO_FIFO_LEN];
    uint32_t head, tail; // niveau = head - tail
    uint16_t last_audio;
    int pot_raw, joy_x_raw, joy_y_raw; // -1 tant qu'aucune valeur reçue
    uint32_t audio_received;
    uint32_t underruns, overruns;
} adc_input_t;

static adc_input_t s_in;

static void adc_input_reset(adc_input_t *in)
{
    in->head = in->tail = 0;
    in->last_audio = 2048;
    in->pot_raw = in->joy_x_raw = in->joy_y_raw = -1;
    in->audio_received = 0;
    in->underruns = in->overruns = 0;
}

// Lit une trame (si dispo) et la répartit. timeout_ms = 0 -> ne bloque jamais.
static void adc_input_pump(audio_io_t *io, adc_input_t *in, uint32_t timeout_ms)
{
    uint32_t len = 0;
    if (adc_continuous_read(io->adc_cont, in->raw, ADC_FRAME_BYTES, &len, timeout_ms) != ESP_OK || len == 0) {
        return;
    }
    uint32_t n = 0;
    if (adc_continuous_parse_data(io->adc_cont, in->raw, len, in->parsed, &n) != ESP_OK) {
        return;
    }
    for (uint32_t i = 0; i < n; i++) {
        const adc_continuous_data_t *d = &in->parsed[i];
        if (!d->valid) {
            continue;
        }
        if (d->channel == io->audio_channel) {
            if (in->head - in->tail >= AUDIO_FIFO_LEN) {
                in->tail++; // file pleine : on jette le plus vieux
                in->overruns++;
            }
            in->fifo[in->head & (AUDIO_FIFO_LEN - 1)] = (uint16_t)d->raw_data;
            in->head++;
            in->audio_received++;
        } else if (d->channel == io->pot_channel) {
            in->pot_raw = (int)d->raw_data;
        } else if (d->channel == io->joy_x_channel) {
            in->joy_x_raw = (int)d->raw_data;
        } else if (d->channel == io->joy_y_channel) {
            in->joy_y_raw = (int)d->raw_data;
        }
    }
}

// Vide les trames en attente dans le pool du driver (adc_continuous_flush_pool refuse de le faire
// pendant que l'ADC tourne), puis repart d'une file vide.
static void adc_input_drain(audio_io_t *io, adc_input_t *in)
{
    uint32_t len = 0;
    while (adc_continuous_read(io->adc_cont, in->raw, ADC_FRAME_BYTES, &len, 0) == ESP_OK && len > 0) {
    }
    in->head = in->tail = 0;
}

static inline uint16_t adc_input_pop(adc_input_t *in)
{
    uint32_t level = in->head - in->tail;
    if (level > AUDIO_FIFO_HIGH) {
        // L'ADC va un peu plus vite que la sortie : on rattrape d'un coup plutôt que de laisser la
        // latence grimper.
        in->tail = in->head - AUDIO_FIFO_REFILL;
        in->overruns++;
    } else if (level == 0) {
        in->underruns++; // l'ADC est en retard : on répète le dernier échantillon
        return in->last_audio;
    }
    in->last_audio = in->fifo[in->tail & (AUDIO_FIFO_LEN - 1)];
    in->tail++;
    return in->last_audio;
}

// --- Stats affichées par une tâche séparée (cœur 0) : l'affichage série prend plusieurs ms et
// bloquerait la boucle audio s'il était fait dedans. ---
typedef struct {
    float carrier_hz, tonalite_db, playback_rate, input_peak, output_peak;
    const char *etat_cle, *etat_ton, *etat_vit;
    uint32_t clip_count, delay_count, underruns, overruns, max_late_us;
    int fs_out, fs_adc;
    bool ready;
} audio_stats_t;

static audio_stats_t s_stats;

// Dernier instant (ms) où l'entrée audio a dépassé AUDIO_LOUD_THRESHOLD : sert à détecter que le
// DFPlayer s'est tu. Le bruit de fond mesuré à l'arrêt est ~0,008, la voix ~0,2-0,3.
#define AUDIO_LOUD_THRESHOLD 0.04f
static volatile uint32_t s_last_loud_ms;

// État des contrôles publié par la boucle audio, lu par led_task (mots de 32 bits : pas de verrou).
typedef struct {
    volatile float carrier_hz, tonalite_db, playback_rate;
    volatile bool cle_latched, joy_latched, valid;
} led_state_t;

static led_state_t s_led;

// Pilote les 2 LED (clignotement selon l'écart à la cible, fixe une fois verrouillé). Tourne sur le
// cœur 0 : led_start_blinking / led_stop_blinking écrivent dans le log et ne doivent pas bloquer
// la boucle audio.
static void led_task(void *arg)
{
    audio_io_t *io = (audio_io_t *)arg;
    bool led_pot_on_solid = false, led_joy_on_solid = false;
    int led_pot_interval_ms = 0, led_joy_interval_ms = 0; // 0 = pas encore démarré
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(200));
        if (!s_led.valid) {
            continue;
        }

        // LED du potentiomètre : clignote selon l'écart à la clé, fixe une fois verrouillée.
        // (échelle = moitié du parcours total, pas la tolérance -> ça accélère sur tout le tour,
        // pas juste dans la petite fenêtre de tolérance)
        float err_cle = fabsf(s_led.carrier_hz - io->cible_carrier_hz) / F_CLE_FEEDBACK_RANGE_HZ;
        if (err_cle > 1.0f) {
            err_cle = 1.0f;
        }
        if (s_led.cle_latched) {
            if (!led_pot_on_solid) {
                led_stop_blinking(&io->led_pot);
                led_set(&io->led_pot, true);
                led_pot_on_solid = true;
            }
        } else {
            int new_interval_ms = LED_BLINK_FAST_MS + (int)(err_cle * (LED_BLINK_SLOW_MS - LED_BLINK_FAST_MS));
            if (led_pot_interval_ms == 0 || abs(new_interval_ms - led_pot_interval_ms) > 20) {
                led_pot_interval_ms = new_interval_ms;
                led_start_blinking(&io->led_pot, led_pot_interval_ms);
            }
        }

        // LED du joystick : clignote selon la moyenne des écarts tonalité+vitesse, fixe une fois
        // les deux verrouillées.
        float err_ton = fabsf(s_led.tonalite_db - io->cible_tonalite_db) / TONALITE_FEEDBACK_RANGE_DB;
        float err_vit = fabsf(s_led.playback_rate - io->cible_playback_rate) / PLAYBACK_RATE_FEEDBACK_RANGE;
        float err_joy = (err_ton + err_vit) / 2.0f;
        if (err_joy > 1.0f) {
            err_joy = 1.0f;
        }
        if (s_led.joy_latched) {
            if (!led_joy_on_solid) {
                led_stop_blinking(&io->led_joy);
                led_set(&io->led_joy, true);
                led_joy_on_solid = true;
            }
        } else {
            int new_interval_ms = LED_BLINK_FAST_MS + (int)(err_joy * (LED_BLINK_SLOW_MS - LED_BLINK_FAST_MS));
            if (led_joy_interval_ms == 0 || abs(new_interval_ms - led_joy_interval_ms) > 20) {
                led_joy_interval_ms = new_interval_ms;
                led_start_blinking(&io->led_joy, led_joy_interval_ms);
            }
        }
    }
}
static portMUX_TYPE s_stats_mux = portMUX_INITIALIZER_UNLOCKED;

static void log_task(void *arg)
{
    (void)arg;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(500));
        audio_stats_t st;
        taskENTER_CRITICAL(&s_stats_mux);
        st = s_stats;
        s_stats.ready = false;
        taskEXIT_CRITICAL(&s_stats_mux);
        if (!st.ready) {
            continue;
        }
        ESP_LOGI(TAG_AUDIO,
                 "porteuse=%.1fHz(%s)  tonalite=%.1fdB(%s)  vitesse=%.3fx(%s)  "
                 "crete_in=%.3f  crete_out=%.3f  ecretages=%u  retards=%u  "
                 "retard_max=%uus  fs_reel=%dHz  fs_adc=%dHz  manques=%u  surplus=%u",
                 st.carrier_hz, st.etat_cle, st.tonalite_db, st.etat_ton, st.playback_rate, st.etat_vit,
                 st.input_peak, st.output_peak, st.clip_count, st.delay_count, st.max_late_us,
                 st.fs_out, st.fs_adc, st.underruns, st.overruns);
    }
}

typedef struct {
    audio_io_t *io;
} audio_task_args_t;

static void audio_task(void *arg)
{
    audio_io_t *io = ((audio_task_args_t *)arg)->io;

    esp_task_wdt_add(NULL);

    // --- Filtre passe-bande de l'inversion (après le mélange, clé = potentiomètre) ---
    biquad_t bp_hp_1, bp_hp_2, bp_lp_1, bp_lp_2;
    biquad_init_highpass(&bp_hp_1, BANDPASS_HP_FC_HZ, BIQUAD_Q1, FS_HZ);
    biquad_init_highpass(&bp_hp_2, BANDPASS_HP_FC_HZ, BIQUAD_Q2, FS_HZ);
    biquad_init_lowpass(&bp_lp_1, BANDPASS_LP_FC_HZ, BIQUAD_Q1, FS_HZ);
    biquad_init_lowpass(&bp_lp_2, BANDPASS_LP_FC_HZ, BIQUAD_Q2, FS_HZ);

    // --- Même filtre, mais une instance séparée pour le son normal (passe-droit) : évite que l'état
    // du filtre de brouillage ne "fuite" dans le son normal (clic) au moment où la porte bascule.
    // Sans AUCUN filtrage ici, le bruit électrique brut de l'ADC ressort strident.
    biquad_t clean_hp_1, clean_hp_2, clean_lp_1, clean_lp_2;
    biquad_init_highpass(&clean_hp_1, BANDPASS_HP_FC_HZ, BIQUAD_Q1, FS_HZ);
    biquad_init_highpass(&clean_hp_2, BANDPASS_HP_FC_HZ, BIQUAD_Q2, FS_HZ);
    biquad_init_lowpass(&clean_lp_1, BANDPASS_LP_FC_HZ, BIQUAD_Q1, FS_HZ);
    biquad_init_lowpass(&clean_lp_2, BANDPASS_LP_FC_HZ, BIQUAD_Q2, FS_HZ);

    // --- Shelf de la tonalité (clé = joystick X) ---
    biquad_t shelf_bas, shelf_haut;
    biquad_set_shelf(&shelf_bas, true, SHELF_BAS_FC_HZ, 0.0f, FS_HZ);
    biquad_set_shelf(&shelf_haut, false, SHELF_HAUT_FC_HZ, 0.0f, FS_HZ);

    // --- Tampon de relecture à débit variable (clé = joystick Y) ---
    int16_t *speed_buf[SPEED_NUM_CHUNKS];
    for (int c = 0; c < SPEED_NUM_CHUNKS; c++) {
        speed_buf[c] = heap_caps_calloc(SPEED_CHUNK_LEN, sizeof(int16_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        if (speed_buf[c] == NULL) {
            ESP_LOGE(TAG_AUDIO, "Pas assez de RAM pour le tampon de vitesse (bloc %d/%d, libre=%u, plus grand bloc=%u)",
                     c + 1, SPEED_NUM_CHUNKS, (unsigned)heap_caps_get_free_size(MALLOC_CAP_8BIT),
                     (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
            vTaskDelete(NULL);
            return;
        }
    }
    ESP_LOGI(TAG_AUDIO, "Tampon de vitesse: %d echantillons (%.2f s), RAM libre restante=%u",
             SPEED_BUF_LEN, (float)SPEED_BUF_LEN / FS_HZ, (unsigned)heap_caps_get_free_size(MALLOC_CAP_8BIT));
    uint32_t write_index = 0;
    float read_pos = 0.0f;

    float center = 2048.0f;

    // Important : on part de la VRAIE position actuelle des contrôles, pas de 0. Sinon, pendant la
    // première seconde, la valeur lissée "balaie" artificiellement du minimum vers la vraie position
    // -> elle peut traverser la cible par hasard et se verrouiller tout de suite, sans que le joueur
    // touche à rien.
    adc_input_t *in = &s_in;
    adc_input_reset(in);
    adc_input_drain(io, in);
    while (in->pot_raw < 0 || in->joy_x_raw < 0 || in->joy_y_raw < 0) {
        adc_input_pump(io, in, 100);
    }
    float pot_smooth = (float)in->pot_raw / 4095.0f;
    float joy_x_smooth = (float)in->joy_x_raw / 4095.0f; // 0..1
    float joy_y_smooth = (float)in->joy_y_raw / 4095.0f; // 0..1

    float carrier_hz = CARRIER_MIN_HZ + pot_smooth * (CARRIER_MAX_HZ - CARRIER_MIN_HZ);
    float tonalite_db = TONALITE_DB_MIN + joy_x_smooth * (TONALITE_DB_MAX - TONALITE_DB_MIN);
    float playback_rate = PLAYBACK_RATE_MIN + joy_y_smooth * (PLAYBACK_RATE_MAX - PLAYBACK_RATE_MIN);
    uint32_t phase = 0;

    float input_peak = 0.0f;
    float output_peak = 0.0f;
    uint32_t delay_count = 0;
    uint32_t clip_count = 0;
    uint32_t max_late_us = 0;
    float output_gain = OUTPUT_GAIN;
    // "_ok" = pile dessus en ce moment ; "_latched" = déjà trouvé une fois -> reste vrai pour le
    // reste de la partie (comme une porte déverrouillée, pas besoin de tenir la clé).
    bool cle_ok = false, tonalite_ok = false, vitesse_ok = false;
    bool cle_latched = false, tonalite_latched = false, vitesse_latched = false;
    int cle_ok_streak = 0, tonalite_ok_streak = 0, vitesse_ok_streak = 0;

    ESP_LOGI(TAG_AUDIO, "Mesure du centre de l'entree audio (2000 echantillons)...");
    {
        int64_t sum = 0;
        int count = 0;
        while (count < 2000) {
            if (in->head == in->tail) {
                adc_input_pump(io, in, 100);
                continue;
            }
            sum += adc_input_pop(in);
            count++;
        }
        center = (float)sum / (float)count;
    }
    ESP_LOGI(TAG_AUDIO, "Centre audio mesure: %.1f", center);

    // On repart d'une file vide puis on la pré-remplit un peu, pour démarrer avec une marge.
    adc_input_drain(io, in);
    adc_input_reset(in);
    while (in->head - in->tail < AUDIO_FIFO_REFILL * 2) {
        adc_input_pump(io, in, 100);
    }
    uint32_t report_audio_received = in->audio_received;

    int64_t schedule_start_us = esp_timer_get_time();
    int64_t schedule_index = 0; // nb d'échantillons depuis schedule_start_us
    int64_t next_sample_us = schedule_start_us;
    int64_t last_report_us = next_sample_us;
    int sample_counter = 0;
    int report_sample_counter = 0;

    const int ctrl_read_every_n_samples = FS_HZ / 100; // ~10 ms
    const int led_update_every_n_samples = FS_HZ / 5;  // ~200 ms (évite de redémarrer le timer trop souvent)

    while (1) {
        int64_t now_us = esp_timer_get_time();
        if (now_us < next_sample_us) {
            while (esp_timer_get_time() < next_sample_us) {
                // attente active
            }
        } else if (now_us > next_sample_us) {
            delay_count++;
            uint32_t late_us = (uint32_t)(now_us - next_sample_us);
            if (late_us > max_late_us) {
                max_late_us = late_us;
            }
            // Petit retard (lecture des contrôles toutes les 10 ms, affichage du log toutes les 0,5 s) :
            // on enchaîne sans attendre pour le rattraper, sinon le taux réel tombe sous FS_HZ.
            // Gros retard (> 20 ms) : on resynchronise.
            if (now_us - next_sample_us > 20000) {
                schedule_start_us = now_us;
                schedule_index = 0;
            }
        }
        schedule_index++;
        next_sample_us = schedule_start_us + (schedule_index * 1000000LL) / FS_HZ;

        if (in->head - in->tail < AUDIO_FIFO_REFILL) {
            adc_input_pump(io, in, 0);
        }

        if (sample_counter % ctrl_read_every_n_samples == 0) {
            int pot_raw = in->pot_raw, joy_x_raw = in->joy_x_raw, joy_y_raw = in->joy_y_raw;

            float pot_norm = (float)pot_raw / 4095.0f;
            pot_smooth += CTRL_SMOOTH_ALPHA * (pot_norm - pot_smooth);
            carrier_hz = CARRIER_MIN_HZ + pot_smooth * (CARRIER_MAX_HZ - CARRIER_MIN_HZ);
            cle_ok = fabsf(carrier_hz - io->cible_carrier_hz) < F_CLE_TOLERANCE_HZ;
            cle_ok_streak = cle_ok ? (cle_ok_streak + 1) : 0;
            if (cle_ok_streak >= OK_STREAK_REQUIRED) {
                cle_latched = true;
            }

            float joy_x_norm = (float)joy_x_raw / 4095.0f;
            joy_x_smooth += CTRL_SMOOTH_ALPHA * (joy_x_norm - joy_x_smooth);
            tonalite_db = TONALITE_DB_MIN + joy_x_smooth * (TONALITE_DB_MAX - TONALITE_DB_MIN);
            tonalite_ok = fabsf(tonalite_db - io->cible_tonalite_db) < TONALITE_TOLERANCE_DB;
            tonalite_ok_streak = tonalite_ok ? (tonalite_ok_streak + 1) : 0;
            if (tonalite_ok_streak >= OK_STREAK_REQUIRED) {
                tonalite_latched = true;
            }
            biquad_set_shelf(&shelf_bas, true, SHELF_BAS_FC_HZ, -tonalite_db, FS_HZ);
            biquad_set_shelf(&shelf_haut, false, SHELF_HAUT_FC_HZ, tonalite_db, FS_HZ);

            float joy_y_norm = (float)joy_y_raw / 4095.0f;
            joy_y_smooth += CTRL_SMOOTH_ALPHA * (joy_y_norm - joy_y_smooth);
            playback_rate = PLAYBACK_RATE_MIN + joy_y_smooth * (PLAYBACK_RATE_MAX - PLAYBACK_RATE_MIN);
            vitesse_ok = fabsf(playback_rate - io->cible_playback_rate) < PLAYBACK_RATE_TOLERANCE;
            vitesse_ok_streak = vitesse_ok ? (vitesse_ok_streak + 1) : 0;
            if (vitesse_ok_streak >= OK_STREAK_REQUIRED) {
                vitesse_latched = true;
            }
        }

        if (sample_counter % led_update_every_n_samples == 0) {
            // On publie seulement l'état : les LED sont pilotées par led_task sur le cœur 0.
            // (led_start_blinking écrit dans le log série, ~5 ms bloquantes : appelé ici, chaque
            // changement de cadence faisait un trou dans le son -> claquements en rafale.)
            s_led.carrier_hz = carrier_hz;
            s_led.tonalite_db = tonalite_db;
            s_led.playback_rate = playback_rate;
            s_led.cle_latched = cle_latched;
            s_led.joy_latched = tonalite_latched && vitesse_latched;
            s_led.valid = true;
        }

        // 1. Lecture brute + centrage
        int raw = adc_input_pop(in);
        center += CENTER_TRACK_ALPHA * ((float)raw - center);
        float x = ((float)raw - center) / 2048.0f;
        if (fabsf(x) > input_peak) {
            input_peak = fabsf(x);
        }
        if (fabsf(x) > AUDIO_LOUD_THRESHOLD) {
            s_last_loud_ms = (uint32_t)(now_us / 1000);
        }

        // Porte : les 3 verrouillées -> son normal (passe-droit), et ça reste vrai même si on bouge
        // les contrôles après (une fois trouvé, pas besoin de tenir la position). Sinon -> brouillé.
        bool all_ok = cle_latched && tonalite_latched && vitesse_latched;
        float target_gain = all_ok ? OUTPUT_GAIN * SOLVED_VOLUME_BOOST : OUTPUT_GAIN;
        output_gain += GAIN_RAMP_ALPHA * (target_gain - output_gain);

        float y;
        if (all_ok) {
            // Même filtre passe-bande que le brouillage (pas de mélange/shelf), juste pour enlever
            // le bruit électrique hors de la bande vocale -> évite le bruit strident.
            y = biquad_process(&clean_hp_1, x);
            y = biquad_process(&clean_hp_2, y);
            y = biquad_process(&clean_lp_1, y);
            y = biquad_process(&clean_lp_2, y);
        } else {
            // 2. Vitesse : on écrit l'échantillon courant, on relit à un débit fractionnaire (joystick Y)
            SPEED_AT(speed_buf, write_index) = (int16_t)(x * 32767.0f);
            write_index = (write_index + 1) & (SPEED_BUF_LEN - 1);

            uint32_t idx0 = (uint32_t)read_pos & (SPEED_BUF_LEN - 1);
            uint32_t idx1 = (idx0 + 1) & (SPEED_BUF_LEN - 1);
            float frac = read_pos - (float)(uint32_t)read_pos;
            float x_speed = ((float)SPEED_AT(speed_buf, idx0) / 32767.0f) * (1.0f - frac)
                           + ((float)SPEED_AT(speed_buf, idx1) / 32767.0f) * frac;
            read_pos += playback_rate;
            if (read_pos >= (float)SPEED_BUF_LEN) {
                read_pos -= (float)SPEED_BUF_LEN;
            }
            if (read_pos < 0.0f) {
                read_pos += (float)SPEED_BUF_LEN;
            }

            // 3. Inversion (potentiomètre = clé) : mélange par la porteuse, puis filtre passe-bande [300-3000Hz]
            uint32_t step = (uint32_t)((carrier_hz / (float)FS_HZ) * 4294967296.0f);
            phase += step;
            uint32_t index = phase >> CARRIER_TABLE_SHIFT;
            float carrier = carrier_table[index & (CARRIER_TABLE_SIZE - 1)];

            float mixed = 2.0f * carrier * x_speed;

            y = biquad_process(&bp_hp_1, mixed);
            y = biquad_process(&bp_hp_2, y);
            y = biquad_process(&bp_lp_1, y);
            y = biquad_process(&bp_lp_2, y);

            // 4. Tonalité (joystick X) : shelf bas puis shelf haut
            y = biquad_process(&shelf_bas, y);
            y = biquad_process(&shelf_haut, y);
        }

        float v = 128.0f + output_gain * y * 127.0f;
        if (fabsf(y) > output_peak) {
            output_peak = fabsf(y);
        }
        if (v < 0.0f) { v = 0.0f; clip_count++; }
        if (v > 255.0f) { v = 255.0f; clip_count++; }
        dac_oneshot_output_voltage(io->dac_handle, (uint8_t)v);

        sample_counter++;
        if (now_us - last_report_us >= 500000) {
            // Une fois verrouillé, l'état affiché reste "LOCK" pour toujours (même si le réglage
            // dérive après coup) : pas de retour en arrière visuel une fois trouvé.
            const char *etat_cle = cle_latched ? "LOCK" : (cle_ok ? "OK" : "--");
            const char *etat_ton = tonalite_latched ? "LOCK" : (tonalite_ok ? "OK" : "--");
            const char *etat_vit = vitesse_latched ? "LOCK" : (vitesse_ok ? "OK" : "--");
            int64_t dt_us = now_us - last_report_us;
            taskENTER_CRITICAL(&s_stats_mux);
            s_stats.carrier_hz = carrier_hz;
            s_stats.tonalite_db = tonalite_db;
            s_stats.playback_rate = playback_rate;
            s_stats.etat_cle = etat_cle;
            s_stats.etat_ton = etat_ton;
            s_stats.etat_vit = etat_vit;
            s_stats.input_peak = input_peak;
            s_stats.output_peak = output_peak;
            s_stats.clip_count = clip_count;
            s_stats.delay_count = delay_count;
            s_stats.max_late_us = max_late_us;
            s_stats.underruns = in->underruns;
            s_stats.overruns = in->overruns;
            s_stats.fs_out = (int)((int64_t)(sample_counter - report_sample_counter) * 1000000LL / dt_us);
            s_stats.fs_adc = (int)((int64_t)(in->audio_received - report_audio_received) * 1000000LL / dt_us);
            s_stats.ready = true;
            taskEXIT_CRITICAL(&s_stats_mux);
            report_sample_counter = sample_counter;
            report_audio_received = in->audio_received;
            in->underruns = 0;
            in->overruns = 0;
            max_late_us = 0;
            delay_count = 0; // compté par période de rapport (0,5 s), plus facile à lire
            input_peak = 0.0f;
            output_peak = 0.0f;
            clip_count = 0;
            last_report_us = now_us;
            esp_task_wdt_reset();
        }
    }
}

static void df_event_cb(const dfplayer_mini_event_t *event, void *ctx)
{
    (void)ctx;
    switch (event->type) {
        case DFPLAYER_MINI_EVT_DEVICE_ONLINE:
            ESP_LOGI(TAG, "EVT device online, mask=0x%04X", event->param);
            break;
        case DFPLAYER_MINI_EVT_DEVICE_OFFLINE:
            ESP_LOGW(TAG, "EVT device offline, mask=0x%04X", event->param);
            break;
        case DFPLAYER_MINI_EVT_TF_TRACK_DONE:
            ESP_LOGI(TAG, "EVT TF track done, track=%d", event->param);
            break;
        case DFPLAYER_MINI_EVT_INIT_DONE:
            ESP_LOGI(TAG, "EVT init done, mask=0x%04X", event->param);
            break;
        case DFPLAYER_MINI_EVT_ERROR:
            ESP_LOGE(TAG, "EVT ERROR code=0x%04X", event->param);
            break;
        case DFPLAYER_MINI_EVT_ACK:
            ESP_LOGI(TAG, "EVT ACK");
            break;
        default:
            ESP_LOGI(TAG, "EVT type=0x%02X param=0x%04X", event->type, event->param);
            break;
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "INIT DU MAIN");

    static audio_io_t io;
    audio_io_init(&io);
    carrier_table_init();

    static audio_task_args_t audio_args;
    audio_args.io = &io;
    xTaskCreatePinnedToCore(audio_task, "audio_decode_task", 4096, &audio_args, 10, NULL, 1);
    xTaskCreatePinnedToCore(log_task, "audio_log_task", 3072, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(led_task, "led_task", 3072, &io, 2, NULL, 0);

    dfplayer_mini_config_t cfg = DFPLAYER_MINI_CONFIG_DEFAULT();
    cfg.tx_gpio    = TX_GPIO;
    cfg.rx_gpio    = RX_GPIO;
    cfg.busy_gpio  = DFPLAYER_BUSY_GPIO;
    cfg.cmd_gap_ms = 150;
    cfg.event_cb   = df_event_cb;

    dfplayer_mini_handle_t mp3;
    ESP_ERROR_CHECK(dfplayer_mini_create(&cfg, &mp3));
    ESP_LOGI(TAG, "ESP GPIO%d -> DFPlayer RX, DFPlayer TX -> ESP GPIO%d", TX_GPIO, RX_GPIO);

    ESP_LOGI(TAG, "begin(): wait for real 0x3F init event (up to 3000ms)...");
    esp_err_t err = dfplayer_mini_begin(mp3, 30, 3000);
    ESP_LOGI(TAG, "begin() returned: %s", esp_err_to_name(err));

    uint16_t file_count = 0;
    err = dfplayer_mini_query_tf_file_count(mp3, &file_count, 2000);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "SD détectée: %d fichier(s)", file_count);
    } else {
        ESP_LOGE(TAG, "query_tf_file_count a échoué: %s (pas de réponse du module — vérifier GND commun / câblage UART)",
                 esp_err_to_name(err));
    }

    uint16_t status = 0;
    err = dfplayer_mini_query_status(mp3, &status, 1000);
    ESP_LOGI(TAG, "query_status: %s, status=0x%04X", esp_err_to_name(err), status);

    ESP_LOGI(TAG, "play folder 01 track 001");
    ESP_ERROR_CHECK(dfplayer_mini_play_folder_track(mp3, 1, 1));

    // 1) Le DFPlayer boucle lui-même sur la piste (CMD 0x19). Avant, on attendait l'événement
    //    "piste terminée" (0x3D) pour relancer, mais sur ce montage il n'arrive jamais (la liaison
    //    DFPlayer TX -> ESP32 est peu fiable : pas de 0x3F au boot, "0 fichier" alors que ça joue)
    //    -> la lecture s'arrêtait pour de bon à la fin de la piste (~32 s).
    vTaskDelay(pdMS_TO_TICKS(200));
    dfplayer_mini_loop_current(mp3, true);

    // 2) Filet de sécurité : si l'entrée audio reste silencieuse plus de SILENCE_RESTART_MS (module
    //    qui a redémarré sur une chute d'alimentation, commande perdue...), on relance. La broche BUSY
    //    n'est pas utilisable ici : elle indique "arrêté" même pendant la lecture.
    const int poll_ms = 100;
    const uint32_t SILENCE_RESTART_MS = 4000;
    int restart_count = 0;
    int heartbeat_ms = 0;
    s_last_loud_ms = (uint32_t)(esp_timer_get_time() / 1000);
    while (1) {
        dfplayer_mini_event_t ev;
        if (dfplayer_mini_wait_event(mp3, &ev, poll_ms) == ESP_OK) {
            if (ev.type == DFPLAYER_MINI_EVT_TF_TRACK_DONE) {
                dfplayer_mini_play_folder_track(mp3, 1, 1);
                dfplayer_mini_loop_current(mp3, true);
            }
        }

        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
        uint32_t silent_ms = now_ms - s_last_loud_ms;
        if (silent_ms >= SILENCE_RESTART_MS) {
            restart_count++;
            ESP_LOGW(TAG, "Aucun son du DFPlayer depuis %u ms -> relance de la piste (relance #%d)",
                     (unsigned)silent_ms, restart_count);
            dfplayer_mini_play_folder_track(mp3, 1, 1);
            vTaskDelay(pdMS_TO_TICKS(200));
            dfplayer_mini_loop_current(mp3, true);
            s_last_loud_ms = (uint32_t)(esp_timer_get_time() / 1000); // laisse le temps de redémarrer
        }

        heartbeat_ms += poll_ms;
        if (heartbeat_ms >= 3000) {
            heartbeat_ms = 0;
            ESP_LOGI(TAG, "listening for audio... silence=%ums relances=%d", (unsigned)silent_ms, restart_count);
        }
    }
}
