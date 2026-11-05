/**
 * La LED est branchée sur un GPIO (avec une résistance qui limite le courant), piloté en PWM par le
 * périphérique LEDC.
 *
 * Pourquoi du PWM plutôt qu'un simple GPIO tout-ou-rien : allumer la LED d'un coup fait baisser le
 * 3,3 V d'environ 0,1 % (mesuré). Le DAC audio de l'ESP32 est proportionnel au 3,3 V, donc chaque
 * clignotement faisait un petit "toc" dans le haut-parleur (bruit de fond qui "bat"). Ici la LED
 * monte et descend en fondu (LED_FADE_MS) : le courant change progressivement, sans saut audible.
 * La fréquence PWM (40 kHz) est au-dessus de l'audible.
 *
 * Pour la faire clignoter sans occuper une tâche, led_start_blinking utilise un esp_timer : une
 * fonction appelée automatiquement à intervalle régulier, qui inverse l'état de la LED.
 */

#include "led.h"
#include "driver/ledc.h"
#include "esp_log.h"

#define LED_LEDC_MODE      LEDC_HIGH_SPEED_MODE
#define LED_LEDC_TIMER     LEDC_TIMER_1 // LEDC_TIMER_0 est réservé à components/buzzer_passive
#define LED_LEDC_RES       LEDC_TIMER_10_BIT
#define LED_LEDC_FREQ_HZ   40000
#define LED_DUTY_ON        ((1 << 10) - 1)
#define LED_FADE_MS        40

static const char *TAG = "components/led";
static int s_next_channel = 0;
static bool s_ledc_ready = false;

static void ledc_init_once(void)
{
    if (s_ledc_ready) {
        return;
    }
    ledc_timer_config_t timer = {
        .speed_mode = LED_LEDC_MODE,
        .duty_resolution = LED_LEDC_RES,
        .timer_num = LED_LEDC_TIMER,
        .freq_hz = LED_LEDC_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));
    ESP_ERROR_CHECK(ledc_fade_func_install(0));
    s_ledc_ready = true;
}

led_t led_init(gpio_num_t gpio)
{
    ESP_LOGI(TAG, "Configuration du GPIO %d en PWM (LEDC canal %d)", gpio, s_next_channel);
    ledc_init_once();

    ledc_channel_t channel = (ledc_channel_t)s_next_channel++;
    ledc_channel_config_t ch = {
        .gpio_num = gpio,
        .speed_mode = LED_LEDC_MODE,
        .channel = channel,
        .timer_sel = LED_LEDC_TIMER,
        .duty = 0,
        .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ch));

    return (led_t){ .gpio = gpio, .channel = channel, .is_on = false, .blink_timer = NULL, .fade_ms = LED_FADE_MS };
}

static void led_fade_to(led_t *led, bool on)
{
    // (ledc_fade_stop n'existe pas sur l'ESP32 classique : le fondu dure au plus la moitié de
    // l'intervalle de clignotement, il est donc toujours fini avant le suivant.)
    ledc_set_fade_time_and_start(LED_LEDC_MODE, led->channel, on ? LED_DUTY_ON : 0,
                                 led->fade_ms, LEDC_FADE_NO_WAIT);
}

void led_set(led_t *led, bool on)
{
    if (led == NULL) {
        return;
    }

    led->is_on = on;
    led_fade_to(led, on);
}

static void blink_timer_callback(void *arg)
{
    led_t *led = (led_t *)arg;
    led_set(led, !led->is_on);
}

void led_start_blinking(led_t *led, uint32_t interval_ms)
{
    if (led == NULL || interval_ms == 0) {
        return;
    }

    if (led->blink_timer == NULL) {
        const esp_timer_create_args_t timer_args = {
            .callback = &blink_timer_callback,
            .arg = led,
            .name = "led_blink",
        };
        esp_timer_create(&timer_args, &led->blink_timer);
    }

    esp_timer_stop(led->blink_timer); // Pour réinitialiser le timer s'il était déjà en cours

    // Le fondu doit finir avant le prochain changement d'état.
    led->fade_ms = (interval_ms / 2 < LED_FADE_MS) ? interval_ms / 2 : LED_FADE_MS;

    ESP_LOGI(TAG, "LED (GPIO %d): clignotement démarré", led->gpio);

    led_set(led, true); // Pour allumer la LED immédiatement
    esp_timer_start_periodic(led->blink_timer, (uint64_t)interval_ms * 1000);
}

void led_stop_blinking(led_t *led)
{
    if (led == NULL || led->blink_timer == NULL) {
        return;
    }

    esp_timer_stop(led->blink_timer);
    ESP_LOGI(TAG, "LED (GPIO %d): clignotement arrêté", led->gpio);
    led->fade_ms = LED_FADE_MS;
    led_set(led, false);
}
