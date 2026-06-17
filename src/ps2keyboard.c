/*
  ps2keyboard.c - PS/2 keyboard driver for ESP-IDF.

  Originally derived from the Arduino PS2Keyboard library by
  Christian Weichel, Paul Stoffregen, L. Abraham Smith, Cuningan and
  Lahiru Nirmal. Ported to native ESP-IDF, 2026.

  Released under the GNU Lesser General Public License v2.1.
*/

#include "ps2keyboard.h"

#include <string.h>

#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "ps2keyboard";

#define PS2_BUFFER_SIZE   45
#define PS2_LED_CONTROL   0xED

/* State machine flags for scan-code processing. */
#define PS2_BREAK     0x01
#define PS2_MODIFIER  0x02
#define PS2_SHIFT_L   0x04
#define PS2_SHIFT_R   0x08
#define PS2_ALTGR     0x10

/* LED state bit flags (PS/2 LED command payload). */
#define PS2_LED_SCROLL 0x01
#define PS2_LED_NUM    0x02
#define PS2_LED_CAPS   0x04

static volatile uint8_t s_buffer[PS2_BUFFER_SIZE];
static volatile uint8_t s_head;
static volatile uint8_t s_tail;

/*
  ISR frame state lives at file scope (rather than function-local statics)
  so that ps2_send_byte() can clear it after re-enabling the clock-line
  interrupt. Otherwise a stale bit_count from before the bit-bang can
  corrupt the first scan code that follows an LED update.
*/
static volatile uint8_t  s_isr_bit_count;
static volatile uint8_t  s_isr_incoming;
static volatile uint32_t s_isr_prev_us;

static gpio_num_t s_data_pin  = GPIO_NUM_NC;
static gpio_num_t s_clock_pin = GPIO_NUM_NC;

static const ps2_keymap_t *s_keymap;

static uint8_t s_char_buffer;
static uint8_t s_utf8_next;

static bool    s_caps_lock_on;
static bool    s_num_lock_on;
static bool    s_scroll_lock_on;
static uint8_t s_led_state;

static bool s_isr_service_installed;
static bool s_isr_handler_added;

/* ---------- low-level pin helpers ---------- */

static inline int ps2_read_pin(gpio_num_t pin)
{
    return gpio_get_level(pin);
}

static inline void ps2_write_pin(gpio_num_t pin, int level)
{
    gpio_set_level(pin, level);
}

static inline void ps2_pin_input_pullup(gpio_num_t pin)
{
    gpio_set_direction(pin, GPIO_MODE_INPUT);
    gpio_set_pull_mode(pin, GPIO_PULLUP_ONLY);
}

static inline void ps2_pin_output(gpio_num_t pin)
{
    gpio_set_direction(pin, GPIO_MODE_OUTPUT);
}

static void ps2_busy_wait_us(uint32_t us)
{
    int64_t start = esp_timer_get_time();
    while ((esp_timer_get_time() - start) < (int64_t)us) {
        /* spin */
    }
}

/* Spin-wait until `pin` reaches `target_level`, or until `timeout_us` elapses.
   Returns true on success, false on timeout. */
static bool ps2_wait_for_level(gpio_num_t pin, int target_level, uint32_t timeout_us)
{
    int64_t deadline = esp_timer_get_time() + (int64_t)timeout_us;
    while (gpio_get_level(pin) != target_level) {
        if (esp_timer_get_time() > deadline) {
            return false;
        }
    }
    return true;
}

/* ---------- PS/2 command transmit (host -> keyboard) ---------- */

static bool ps2_parity_odd(uint16_t data)
{
    uint8_t count = 0;
    for (int i = 0; i < 10; i++) {
        count += (data >> i) & 1U;
    }
    return (count % 2U) != 0U;
}

/*
  Bit-bangs one byte (with parity + stop) to the keyboard. The clock-line
  ISR is disabled for the duration so the device-driven clock doesn't
  fire our scan-code handler with garbage.

  Reference: https://karooza.net/how-to-interface-a-ps2-keyboard
*/
/* PS/2 host-to-device framing tops out around 70us per clock half-cycle.
   10ms per transition is comfortably generous and lets us bail out without
   hanging the consumer task if the device stops clocking. */
#define PS2_TX_PIN_TIMEOUT_US 10000U

static esp_err_t ps2_send_byte(uint8_t cmd_byte)
{
    uint16_t cmd = cmd_byte;
    if (!ps2_parity_odd(cmd)) {
        cmd |= 0x0100; /* odd parity bit */
    }
    cmd |= 0x0200; /* stop bit */

    gpio_intr_disable(s_clock_pin);

    /* Pull clock low for ~100us to request transmission. */
    ps2_pin_output(s_clock_pin);
    ps2_pin_output(s_data_pin);
    ps2_write_pin(s_clock_pin, 0);
    ps2_busy_wait_us(100);

    /* Start bit: pull data low, then release clock. */
    ps2_write_pin(s_data_pin, 0);
    ps2_busy_wait_us(20);
    ps2_pin_input_pullup(s_clock_pin);

    esp_err_t err = ESP_OK;

    /* 8 data bits + parity + stop bit. */
    for (int i = 0; i < 10; i++) {
        if (!ps2_wait_for_level(s_clock_pin, 0, PS2_TX_PIN_TIMEOUT_US)) {
            err = ESP_ERR_TIMEOUT;
            goto out;
        }
        ps2_write_pin(s_data_pin, cmd & 0x0001);
        cmd >>= 1;
        if (!ps2_wait_for_level(s_clock_pin, 1, PS2_TX_PIN_TIMEOUT_US)) {
            err = ESP_ERR_TIMEOUT;
            goto out;
        }
    }

    /* Release data line for the device-driven ack. */
    ps2_pin_input_pullup(s_data_pin);

    /* Wait for keyboard ACK: clock low + data low. */
    if (!ps2_wait_for_level(s_clock_pin, 0, PS2_TX_PIN_TIMEOUT_US) ||
        !ps2_wait_for_level(s_data_pin,  0, PS2_TX_PIN_TIMEOUT_US)) {
        err = ESP_ERR_TIMEOUT;
    }

out:
    ps2_pin_input_pullup(s_clock_pin);
    ps2_pin_input_pullup(s_data_pin);

    /* The device-driven clock cycles during this bit-bang were masked, so
       any stale bit_count carried over would corrupt the next genuine scan
       code. Reset before re-arming the ISR. */
    s_isr_bit_count = 0;
    s_isr_incoming  = 0;
    s_isr_prev_us   = (uint32_t)esp_timer_get_time();

    gpio_intr_enable(s_clock_pin);
    return err;
}

static void ps2_update_leds(uint8_t leds)
{
    if (ps2_send_byte(PS2_LED_CONTROL) != ESP_OK) {
        return;
    }
    (void)ps2_send_byte(leds);
}

/* ---------- ISR: clock-line falling edge ---------- */

static void IRAM_ATTR ps2_isr_handler(void *arg)
{
    (void)arg;

    /* 32-bit time math is enough for the 250ms timeout and is cheaper on
       the LX6 than the int64_t subtraction esp_timer_get_time() returns.
       Wraparound (every ~71 minutes) is handled by unsigned subtraction. */
    uint32_t now_us = (uint32_t)esp_timer_get_time();
    int      val    = gpio_get_level(s_data_pin);

    if ((uint32_t)(now_us - s_isr_prev_us) > 250000U) {
        s_isr_bit_count = 0;
        s_isr_incoming  = 0;
    }
    s_isr_prev_us = now_us;

    /* On the start-bit call, bit_count is 0 and (0 - 1) underflows to 0xFF
       which fails the <=7 check. Subsequent calls land bits 0..7 into
       incoming. */
    uint8_t bit_pos = (uint8_t)(s_isr_bit_count - 1);
    if (bit_pos <= 7) {
        s_isr_incoming |= (uint8_t)(val << bit_pos);
    }
    s_isr_bit_count++;

    if (s_isr_bit_count == 11) {
        uint8_t i = (uint8_t)(s_head + 1);
        if (i >= PS2_BUFFER_SIZE) {
            i = 0;
        }
        if (i != s_tail) {
            s_buffer[i] = s_isr_incoming;
            s_head = i;
        }
        s_isr_bit_count = 0;
        s_isr_incoming  = 0;
    }
}

static inline uint8_t ps2_get_scancode(void)
{
    uint8_t i = s_tail;
    if (i == s_head) {
        return 0;
    }
    i++;
    if (i >= PS2_BUFFER_SIZE) {
        i = 0;
    }
    uint8_t c = s_buffer[i];
    s_tail = i;
    return c;
}

/* ---------- scan-code -> ISO-8859-1 decoder ---------- */

static uint8_t ps2_decode_iso8859(void)
{
    static uint8_t state = 0;

    /* Bail safely if a caller polled before ps2keyboard_begin() ran. */
    if (s_keymap == NULL) {
        return 0;
    }

    while (1) {
        uint8_t scan = ps2_get_scancode();
        if (!scan) {
            return 0;
        }

        if (scan == 0xF0) {
            state |= PS2_BREAK;
            continue;
        }
        if (scan == 0xE0) {
            state |= PS2_MODIFIER;
            continue;
        }

        if (state & PS2_BREAK) {
            if (scan == 0x12) {
                state &= ~PS2_SHIFT_L;
            } else if (scan == 0x59) {
                state &= ~PS2_SHIFT_R;
            } else if (scan == 0x11 && (state & PS2_MODIFIER)) {
                state &= ~PS2_ALTGR;
            }
            state &= ~(PS2_BREAK | PS2_MODIFIER);
            continue;
        }

        if (scan == 0x12) {
            state |= PS2_SHIFT_L;
            continue;
        }
        if (scan == 0x59) {
            state |= PS2_SHIFT_R;
            continue;
        }
        if (scan == 0x11 && (state & PS2_MODIFIER)) {
            state |= PS2_ALTGR;
        } else if (scan == 0x58) {
            s_caps_lock_on = !s_caps_lock_on;
            s_led_state = s_caps_lock_on
                ? (s_led_state | PS2_LED_CAPS)
                : (s_led_state & (uint8_t)~PS2_LED_CAPS);
            ps2_update_leds(s_led_state);
        } else if (scan == 0x77) {
            s_num_lock_on = !s_num_lock_on;
            s_led_state = s_num_lock_on
                ? (s_led_state | PS2_LED_NUM)
                : (s_led_state & (uint8_t)~PS2_LED_NUM);
            ps2_update_leds(s_led_state);
        } else if (scan == 0x7E) {
            s_scroll_lock_on = !s_scroll_lock_on;
            s_led_state = s_scroll_lock_on
                ? (s_led_state | PS2_LED_SCROLL)
                : (s_led_state & (uint8_t)~PS2_LED_SCROLL);
            ps2_update_leds(s_led_state);
        }

        uint8_t ch = 0;
        if (state & PS2_MODIFIER) {
            switch (scan) {
                case 0x70: ch = PS2_INSERT;     break;
                case 0x6C: ch = PS2_HOME;       break;
                case 0x7D: ch = PS2_PAGEUP;     break;
                case 0x71: ch = PS2_DELETE;     break;
                case 0x69: ch = PS2_END;        break;
                case 0x7A: ch = PS2_PAGEDOWN;   break;
                case 0x75: ch = PS2_UPARROW;    break;
                case 0x6B: ch = PS2_LEFTARROW;  break;
                case 0x72: ch = PS2_DOWNARROW;  break;
                case 0x74: ch = PS2_RIGHTARROW; break;
                case 0x4A: ch = '/';            break;
                case 0x5A: ch = PS2_ENTER;      break;
                default:                        break;
            }
        } else if ((state & PS2_ALTGR) && s_keymap->uses_altgr) {
            if (scan < PS2_KEYMAP_SIZE) {
                ch = s_keymap->altgr[scan];
            }
        } else if (scan < PS2_KEYMAP_SIZE) {
            /* Caps Lock only inverts shift for alphabetic keys (a-z); for
               everything else (digits, punctuation, function keys), Shift
               is the sole modifier. Detect "letter" by inspecting the
               unshifted entry rather than carrying a parallel flag table. */
            bool shift_active = (state & (PS2_SHIFT_L | PS2_SHIFT_R)) != 0;
            uint8_t ns = s_keymap->noshift[scan];
            bool is_alpha = (ns >= 'a' && ns <= 'z');
            bool effective_shift = shift_active ^ (is_alpha && s_caps_lock_on);
            ch = effective_shift ? s_keymap->shift[scan] : ns;
        }

        state &= ~(PS2_BREAK | PS2_MODIFIER);

        if (ch) {
            return ch;
        }
    }
}

/* ---------- public API ---------- */

esp_err_t ps2keyboard_begin(gpio_num_t data_pin,
                            gpio_num_t clock_pin,
                            const ps2_keymap_t *keymap)
{
    if (!GPIO_IS_VALID_GPIO(data_pin) || !GPIO_IS_VALID_GPIO(clock_pin)) {
        return ESP_ERR_INVALID_ARG;
    }

    s_data_pin       = data_pin;
    s_clock_pin      = clock_pin;
    s_keymap         = (keymap != NULL) ? keymap : &ps2_keymap_us;
    s_head           = 0;
    s_tail           = 0;
    s_char_buffer    = 0;
    s_utf8_next      = 0;
    s_caps_lock_on   = false;
    s_num_lock_on    = false;
    s_scroll_lock_on = false;
    s_led_state      = 0;
    s_isr_bit_count  = 0;
    s_isr_incoming   = 0;
    s_isr_prev_us    = 0;

    gpio_config_t data_cfg = {
        .pin_bit_mask = (1ULL << data_pin),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&data_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "gpio_config(data) failed: %s", esp_err_to_name(err));
        return err;
    }

    gpio_config_t clk_cfg = {
        .pin_bit_mask = (1ULL << clock_pin),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_NEGEDGE,
    };
    err = gpio_config(&clk_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "gpio_config(clock) failed: %s", esp_err_to_name(err));
        return err;
    }

    if (!s_isr_service_installed) {
        err = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
        if (err == ESP_OK) {
            s_isr_service_installed = true;
        } else if (err != ESP_ERR_INVALID_STATE) {
            ESP_LOGE(TAG, "gpio_install_isr_service failed: %s", esp_err_to_name(err));
            return err;
        }
    }

    err = gpio_isr_handler_add(clock_pin, ps2_isr_handler, NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "gpio_isr_handler_add failed: %s", esp_err_to_name(err));
        return err;
    }
    s_isr_handler_added = true;

    return ESP_OK;
}

void ps2keyboard_end(void)
{
    if (s_isr_handler_added && s_clock_pin != GPIO_NUM_NC) {
        gpio_isr_handler_remove(s_clock_pin);
        s_isr_handler_added = false;
    }
    s_char_buffer = 0;
    s_utf8_next   = 0;
    s_head        = 0;
    s_tail        = 0;
}

bool ps2keyboard_available(void)
{
    if (s_char_buffer || s_utf8_next) {
        return true;
    }
    s_char_buffer = (uint8_t)ps2_decode_iso8859();
    return s_char_buffer != 0;
}

void ps2keyboard_clear(void)
{
    s_char_buffer = 0;
    s_utf8_next   = 0;
}

uint8_t ps2keyboard_read_scancode(void)
{
    return ps2_get_scancode();
}

int ps2keyboard_read(void)
{
    uint8_t result = s_utf8_next;
    if (result) {
        s_utf8_next = 0;
    } else {
        result = s_char_buffer;
        if (result) {
            s_char_buffer = 0;
        } else {
            result = (uint8_t)ps2_decode_iso8859();
        }
        if (result >= 128) {
            s_utf8_next = (uint8_t)((result & 0x3F) | 0x80);
            result      = (uint8_t)(((result >> 6) & 0x1F) | 0xC0);
        }
    }
    if (!result) {
        return -1;
    }
    return result;
}

int ps2keyboard_read_unicode(void)
{
    int result = s_char_buffer;
    if (!result) {
        result = (uint8_t)ps2_decode_iso8859();
    }
    if (!result) {
        return -1;
    }
    s_utf8_next   = 0;
    s_char_buffer = 0;
    return result;
}
