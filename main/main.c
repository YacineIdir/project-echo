#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_task_wdt.h"
#include "esp_adc/adc_oneshot.h"
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
#define FS_HZ             16000
#define SAMPLE_PERIOD_US  (1000000 / FS_HZ)

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
#define SPEED_BUF_SECONDS   3.0f // backlog max avant que le tampon ne boucle (voir note dans le résumé)
#define SPEED_BUF_LEN       ((int)(SPEED_BUF_SECONDS * FS_HZ))
#define PLAYBACK_RATE_MIN   0.5f
#define PLAYBACK_RATE_MAX   1.5f
#define PLAYBACK_RATE_MARGIN 0.1f
#define PLAYBACK_RATE_TOLERANCE 0.06f // élargi pour être plus clément
#define PLAYBACK_RATE_NEUTRE 1.0f // position de repos du joystick (centré) -> la cible ne doit pas tomber ici
#define PLAYBACK_RATE_DEADZONE 0.15f
#define PLAYBACK_RATE_FEEDBACK_RANGE ((PLAYBACK_RATE_MAX - PLAYBACK_RATE_MIN) / 2.0f) // échelle du clignotement

// --- Lissages / gains ---
#define CENTER_TRACK_ALPHA 0.0005f
#define CTRL_SMOOTH_ALPHA  0.05f
// Il faut rester dans la tolérance pendant ce nombre de lectures consécutives (~10ms chacune) avant
// que ça verrouille -> évite qu'un bruit ADC ou un passage transitoire d'une fraction de seconde
// verrouille par accident. ~0.8s ici.
#define OK_STREAK_REQUIRED 80
#define OUTPUT_GAIN        1.0f // Trop haut (ex: 3.0) sature le DAC en permanence -> bruit d'écrêtage

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
    adc_oneshot_unit_handle_t adc_handle;
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
    int16_t *speed_buf = calloc(SPEED_BUF_LEN, sizeof(int16_t));
    if (speed_buf == NULL) {
        ESP_LOGE(TAG_AUDIO, "Pas assez de RAM pour le tampon de vitesse (%d echantillons)", SPEED_BUF_LEN);
        vTaskDelete(NULL);
        return;
    }
    uint32_t write_index = 0;
    float read_pos = 0.0f;

    float center = 2048.0f;

    // Important : on part de la VRAIE position actuelle des contrôles, pas de 0. Sinon, pendant la
    // première seconde, la valeur lissée "balaie" artificiellement du minimum vers la vraie position
    // -> elle peut traverser la cible par hasard et se verrouiller tout de suite, sans que le joueur
    // touche à rien.
    int init_pot_raw = 0, init_joy_x_raw = 0, init_joy_y_raw = 0;
    adc_oneshot_read(io->adc_handle, io->pot_channel, &init_pot_raw);
    adc_oneshot_read(io->adc_handle, io->joy_x_channel, &init_joy_x_raw);
    adc_oneshot_read(io->adc_handle, io->joy_y_channel, &init_joy_y_raw);
    float pot_smooth = (float)init_pot_raw / 4095.0f;
    float joy_x_smooth = (float)init_joy_x_raw / 4095.0f; // 0..1
    float joy_y_smooth = (float)init_joy_y_raw / 4095.0f; // 0..1

    float carrier_hz = CARRIER_MIN_HZ + pot_smooth * (CARRIER_MAX_HZ - CARRIER_MIN_HZ);
    float tonalite_db = TONALITE_DB_MIN + joy_x_smooth * (TONALITE_DB_MAX - TONALITE_DB_MIN);
    float playback_rate = PLAYBACK_RATE_MIN + joy_y_smooth * (PLAYBACK_RATE_MAX - PLAYBACK_RATE_MIN);
    uint32_t phase = 0;

    float input_peak = 0.0f;
    float output_peak = 0.0f;
    uint32_t delay_count = 0;
    uint32_t clip_count = 0;
    // "_ok" = pile dessus en ce moment ; "_latched" = déjà trouvé une fois -> reste vrai pour le
    // reste de la partie (comme une porte déverrouillée, pas besoin de tenir la clé).
    bool cle_ok = false, tonalite_ok = false, vitesse_ok = false;
    bool cle_latched = false, tonalite_latched = false, vitesse_latched = false;
    int cle_ok_streak = 0, tonalite_ok_streak = 0, vitesse_ok_streak = 0;
    bool led_pot_on_solid = false, led_joy_on_solid = false;
    int led_pot_interval_ms = LED_BLINK_SLOW_MS;
    int led_joy_interval_ms = LED_BLINK_SLOW_MS;

    ESP_LOGI(TAG_AUDIO, "Mesure du centre de l'entree audio (2000 echantillons)...");
    center = measure_channel_average(io, io->audio_channel, 2000);
    ESP_LOGI(TAG_AUDIO, "Centre audio mesure: %.1f", center);

    int64_t next_sample_us = esp_timer_get_time();
    int64_t last_report_us = next_sample_us;
    int sample_counter = 0;

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
            next_sample_us = now_us; // resynchronisation
        }
        next_sample_us += SAMPLE_PERIOD_US;

        if (sample_counter % ctrl_read_every_n_samples == 0) {
            int pot_raw = 0, joy_x_raw = 0, joy_y_raw = 0;
            adc_oneshot_read(io->adc_handle, io->pot_channel, &pot_raw);
            adc_oneshot_read(io->adc_handle, io->joy_x_channel, &joy_x_raw);
            adc_oneshot_read(io->adc_handle, io->joy_y_channel, &joy_y_raw);

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
            // LED du potentiomètre : clignote selon l'écart à la clé, fixe une fois verrouillée.
            // (échelle = moitié du parcours total, pas la tolérance -> ça accélère sur tout le tour,
            // pas juste dans la petite fenêtre de tolérance)
            float err_cle = fabsf(carrier_hz - io->cible_carrier_hz) / F_CLE_FEEDBACK_RANGE_HZ;
            if (err_cle > 1.0f) {
                err_cle = 1.0f;
            }
            if (cle_latched) {
                if (!led_pot_on_solid) {
                    led_stop_blinking(&io->led_pot);
                    led_set(&io->led_pot, true);
                    led_pot_on_solid = true;
                }
            } else {
                int new_interval_ms = LED_BLINK_FAST_MS + (int)(err_cle * (LED_BLINK_SLOW_MS - LED_BLINK_FAST_MS));
                if (abs(new_interval_ms - led_pot_interval_ms) > 20) {
                    led_pot_interval_ms = new_interval_ms;
                    led_start_blinking(&io->led_pot, led_pot_interval_ms);
                }
            }

            // LED du joystick : clignote selon la moyenne des écarts tonalité+vitesse, fixe une fois
            // les deux verrouillées.
            float err_ton = fabsf(tonalite_db - io->cible_tonalite_db) / TONALITE_FEEDBACK_RANGE_DB;
            float err_vit = fabsf(playback_rate - io->cible_playback_rate) / PLAYBACK_RATE_FEEDBACK_RANGE;
            float err_joy = (err_ton + err_vit) / 2.0f;
            if (err_joy > 1.0f) {
                err_joy = 1.0f;
            }
            bool want_solid = tonalite_latched && vitesse_latched;
            if (want_solid) {
                if (!led_joy_on_solid) {
                    led_stop_blinking(&io->led_joy);
                    led_set(&io->led_joy, true);
                    led_joy_on_solid = true;
                }
            } else {
                int new_interval_ms = LED_BLINK_FAST_MS + (int)(err_joy * (LED_BLINK_SLOW_MS - LED_BLINK_FAST_MS));
                if (abs(new_interval_ms - led_joy_interval_ms) > 20) {
                    led_joy_interval_ms = new_interval_ms;
                    led_start_blinking(&io->led_joy, led_joy_interval_ms);
                }
            }
        }

        // 1. Lecture brute + centrage
        int raw = 0;
        adc_oneshot_read(io->adc_handle, io->audio_channel, &raw);
        center += CENTER_TRACK_ALPHA * ((float)raw - center);
        float x = ((float)raw - center) / 2048.0f;
        if (fabsf(x) > input_peak) {
            input_peak = fabsf(x);
        }

        // Porte : les 3 verrouillées -> son normal (passe-droit), et ça reste vrai même si on bouge
        // les contrôles après (une fois trouvé, pas besoin de tenir la position). Sinon -> brouillé.
        bool all_ok = cle_latched && tonalite_latched && vitesse_latched;

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
            speed_buf[write_index] = (int16_t)(x * 32767.0f);
            write_index = (write_index + 1) % SPEED_BUF_LEN;

            uint32_t idx0 = (uint32_t)read_pos;
            uint32_t idx1 = (idx0 + 1) % SPEED_BUF_LEN;
            float frac = read_pos - (float)idx0;
            float x_speed = ((float)speed_buf[idx0] / 32767.0f) * (1.0f - frac)
                           + ((float)speed_buf[idx1] / 32767.0f) * frac;
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

        float v = 128.0f + OUTPUT_GAIN * y * 127.0f;
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
            ESP_LOGI(TAG_AUDIO,
                     "porteuse=%.1fHz(%s)  tonalite=%.1fdB(%s)  vitesse=%.3fx(%s)  "
                     "crete_in=%.3f  crete_out=%.3f  ecretages=%u  retards=%u",
                     carrier_hz, etat_cle, tonalite_db, etat_ton, playback_rate, etat_vit,
                     input_peak, output_peak, clip_count, delay_count);
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
    // Boucle native du DFPlayer : plus fiable que de relancer manuellement sur l'évènement
    // "piste terminée" (qui peut être manqué/retardé). Gardé comme filet de sécurité ci-dessous.
    err = dfplayer_mini_loop_current(mp3, true);
    ESP_LOGI(TAG, "loop_current: %s", esp_err_to_name(err));

    while (1) {
        dfplayer_mini_event_t ev;
        if (dfplayer_mini_wait_event(mp3, &ev, 3000) == ESP_OK) {
            if (ev.type == DFPLAYER_MINI_EVT_TF_TRACK_DONE && !dfplayer_mini_is_playing(mp3)) {
                dfplayer_mini_play_folder_track(mp3, 1, 1); // filet de sécurité si la boucle native s'arrête
            }
        } else {
            ESP_LOGI(TAG, "listening for audio... busy=%d", dfplayer_mini_is_playing(mp3));
        }
    }
}
