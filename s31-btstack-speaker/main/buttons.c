/*
 * Speaker buttons, active low to GND (internal pull-ups), polled every 10 ms
 * with a 30 ms debounce:
 *   GPIO 36  Auracast on/off (toggle; off = phone audio plays only here)
 *   GPIO 38  volume up     (repeats while held)
 *   GPIO 40  volume down   (repeats while held)
 *   GPIO 42  Bluetooth pairing (discoverable + connectable for 3 min)
 *   GPIO 44  play / pause  (AVRCP to the phone)
 */
#include "buttons.h"

#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"

#include "a2dp_in.h"
#include "party.h"
#include "power.h"

static const char *TAG = "BTN";

typedef enum { B_AURACAST, B_VOL_UP, B_VOL_DOWN, B_PAIR, B_PLAY } btn_id_t;
/* grace: after an accepted press the same button is ignored for this long, so a
 * burst of taps does one thing (Auracast needs ~2 s to switch roles). Volume
 * keeps hold-to-repeat; its grace only paces fast taps. */
static const struct { int gpio; const char *name; bool repeat; uint16_t grace_ms; } s_btn[] = {
    [B_AURACAST] = { 36, "Auracast",   false, 2000 },
    [B_VOL_UP]   = { 38, "volume +",   true,  150 },
    [B_VOL_DOWN] = { 40, "volume -",   true,  150 },
    [B_PAIR]     = { 42, "pairing",    false, 3000 },
    [B_PLAY]     = { 44, "play/pause", false, 700 },
};
#define NBTN (sizeof(s_btn) / sizeof(s_btn[0]))
#define POLL_MS     10
#define DEBOUNCE    3            /* polls */
#define REPEAT_WAIT 50           /* 500 ms, then */
#define REPEAT_EVERY 20          /* every 200 ms */

static void press(btn_id_t b)
{
    if (power_is_off()) {
        power_wake();                             /* soft off: any button turns it on */
        return;
    }
    switch (b) {
    case B_AURACAST: party_set_auracast_enabled(!party_auracast_enabled()); break;
    case B_VOL_UP:   a2dp_in_button(A2DP_BTN_VOL_UP); break;
    case B_VOL_DOWN: a2dp_in_button(A2DP_BTN_VOL_DOWN); break;
    case B_PAIR:     a2dp_in_button(A2DP_BTN_PAIRING); break;
    case B_PLAY:     a2dp_in_button(A2DP_BTN_PLAY_PAUSE); break;
    }
}

static void button_task(void *arg)
{
    (void)arg;
    uint8_t cnt[NBTN] = { 0 };
    bool down[NBTN] = { false };
    uint32_t held[NBTN] = { 0 };
    int64_t last_ok[NBTN] = { 0 };
    uint32_t ignored[NBTN] = { 0 };
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(POLL_MS));
        for (unsigned i = 0; i < NBTN; i++) {
            bool pressed = gpio_get_level(s_btn[i].gpio) == 0;
            if (pressed != down[i]) {
                if (++cnt[i] >= DEBOUNCE) {
                    down[i] = pressed;
                    cnt[i] = 0;
                    held[i] = 0;
                    if (pressed) {
                        int64_t now = esp_timer_get_time();
                        if (last_ok[i] != 0 && now - last_ok[i] < (int64_t)s_btn[i].grace_ms * 1000) {
                            ignored[i]++;
                            ESP_LOGW(TAG, "%s pressed again within %u ms - ignored (%lu so far)", s_btn[i].name,
                                     s_btn[i].grace_ms, (unsigned long)ignored[i]);
                        } else {
                            last_ok[i] = now;
                            ESP_LOGW(TAG, "%s pressed (GPIO %d)", s_btn[i].name, s_btn[i].gpio);
                            press((btn_id_t)i);
                        }
                    }
                }
            } else {
                cnt[i] = 0;
                if (down[i] && s_btn[i].repeat) {
                    held[i]++;
                    if (held[i] >= REPEAT_WAIT && (held[i] - REPEAT_WAIT) % REPEAT_EVERY == 0) {
                        last_ok[i] = esp_timer_get_time();
                        press((btn_id_t)i);       /* holding: steady repeat */
                    }
                }
            }
        }
    }
}

void buttons_setup(void)
{
    uint64_t mask = 0;
    for (unsigned i = 0; i < NBTN; i++) {
        mask |= 1ULL << s_btn[i].gpio;
    }
    gpio_config_t io = { .pin_bit_mask = mask, .mode = GPIO_MODE_INPUT, .pull_up_en = GPIO_PULLUP_ENABLE,
                         .pull_down_en = GPIO_PULLDOWN_DISABLE, .intr_type = GPIO_INTR_DISABLE };
    esp_err_t e = gpio_config(&io);
    ESP_LOGW(TAG, "buttons %s: Auracast GPIO 36, vol+ 38, vol- 40, pairing 42, play/pause 44 (active low)",
             e == ESP_OK ? "ready" : esp_err_to_name(e));
    xTaskCreatePinnedToCore(button_task, "buttons", 2048, NULL, 3, NULL, 1);
}
