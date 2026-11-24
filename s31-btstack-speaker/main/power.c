/* Soft power-off (see power.h). */
#include "power.h"

#include "btstack.h"
#include "esp_log.h"
#include "esp_system.h"

#include "tones.h"

static const char *TAG = "POWER";

static volatile bool s_off;
static btstack_timer_source_t s_timer;

static void radio_off(btstack_timer_source_t *ts)
{
    (void)ts;
    hci_power_control(HCI_POWER_OFF);
    ESP_LOGW(TAG, "speaker OFF (radio down) - press any button to turn it on");
}

void power_soft_off(const char *why)
{
    if (s_off) return;
    s_off = true;
    ESP_LOGW(TAG, "powering off: %s", why);
    tones_play("poweroff");
    btstack_run_loop_set_timer_handler(&s_timer, radio_off);
    btstack_run_loop_set_timer(&s_timer, 1800);       /* let the tone play */
    btstack_run_loop_add_timer(&s_timer);
}

bool power_is_off(void) { return s_off; }

/* a clean restart, as a power cycle: BTstack's power-off disconnects every link first, so
 * phones see the speaker go (an abrupt restart left them on a dead link that ignored our
 * call-back page until it timed out) */
static btstack_timer_source_t s_reboot_timer;
static btstack_context_callback_registration_t s_reboot_reg;

static void reboot_now(btstack_timer_source_t *ts)
{
    (void)ts;
    esp_restart();
}

static void reboot_on_bt(void *ctx)
{
    (void)ctx;
    hci_power_control(HCI_POWER_OFF);
    btstack_run_loop_set_timer_handler(&s_reboot_timer, reboot_now);
    btstack_run_loop_set_timer(&s_reboot_timer, 800);
    btstack_run_loop_add_timer(&s_reboot_timer);
}

void power_reboot(void)
{
    s_reboot_reg.callback = reboot_on_bt;
    btstack_run_loop_execute_on_main_thread(&s_reboot_reg);
}

void power_wake(void)
{
    if (s_off) {
        ESP_LOGW(TAG, "button: powering on");
        esp_restart();
    }
}
