# ps2keyboard - ESP-IDF Component

[![License: LGPL v2.1](https://img.shields.io/badge/License-LGPL%20v2.1-blue.svg)](https://www.gnu.org/licenses/lgpl-2.1)
[![Version](https://img.shields.io/badge/version-3.1.0-green.svg)](https://github.com/lahirunirmalx/PS2Keyboard)

A native ESP-IDF component for reading from a PS/2 keyboard on the ESP32
family. Ported from the long-standing Arduino `PS2Keyboard` library.

## Features

- Pure ESP-IDF: uses `driver/gpio`, `esp_timer` and the IDF GPIO ISR service
- Interrupt-driven scan-code capture (`IRAM_ATTR` ISR, runs from IRAM)
- ISO-8859-1 character decoding with Shift / AltGr / Caps / Num / Scroll Lock
- LED control (Caps / Num / Scroll Lock) by bit-banging the host-to-device line
- Plain C API, no C++ runtime required
- Supports any ESP32 variant (S2 / S3 / C3 / C6 / H2 / classic)

## Installation

### Drop-in component

Clone (or submodule) this repository under your project's `components/`
directory:

```bash
mkdir -p components
git clone https://github.com/lahirunirmalx/PS2Keyboard.git components/ps2keyboard
```

### ESP-IDF Component Manager

Add it to your project's `idf_component.yml`:

```yaml
dependencies:
  lahirunirmalx/ps2keyboard:
    git: https://github.com/lahirunirmalx/PS2Keyboard.git
    version: "*"
```

## Wiring

| PS/2 Pin | Signal | Connect To                          |
|----------|--------|-------------------------------------|
| 1        | DATA   | `data_pin` GPIO                     |
| 3        | GND    | GND                                 |
| 4        | VCC    | 5 V (PS/2 keyboards expect 5 V)     |
| 5        | CLOCK  | `clock_pin` GPIO (interrupt source) |

> **Note:** ESP32 GPIOs are 3.3 V. PS/2 keyboards are 5 V open-collector with
> internal pull-ups; in practice the lines idle high and the keyboard sinks
> them low, so most modern ESP32 boards work directly. For long-term
> reliability use a level shifter or a series resistor + clamping diode.

## Quick Start

```c
#include "ps2keyboard.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define PS2_DATA_PIN   GPIO_NUM_18
#define PS2_CLOCK_PIN  GPIO_NUM_19

void app_main(void)
{
    ESP_ERROR_CHECK(ps2keyboard_begin(PS2_DATA_PIN, PS2_CLOCK_PIN, NULL));

    while (1) {
        if (ps2keyboard_available()) {
            int c = ps2keyboard_read();
            if (c == PS2_ENTER)      printf("\n");
            else if (c == PS2_TAB)   printf("[Tab]");
            else if (c == PS2_ESC)   printf("[ESC]");
            else if (c > 0)          putchar(c);
            fflush(stdout);
        } else {
            vTaskDelay(pdMS_TO_TICKS(5));
        }
    }
}
```

A complete buildable example lives in [examples/simple_test](examples/simple_test/).
Build it with:

```bash
cd examples/simple_test
idf.py set-target esp32
idf.py build flash monitor
```

## API Reference

```c
esp_err_t ps2keyboard_begin(gpio_num_t data_pin,
                            gpio_num_t clock_pin,
                            const ps2_keymap_t *keymap);
void      ps2keyboard_end(void);

bool      ps2keyboard_available(void);
void      ps2keyboard_clear(void);

uint8_t   ps2keyboard_read_scancode(void);  /* raw scan code  */
int       ps2keyboard_read(void);           /* UTF-8 byte     */
int       ps2keyboard_read_unicode(void);   /* code point     */
```

Pass `NULL` as the keymap to use the bundled `ps2_keymap_us`. Custom layouts
can be supplied as `const ps2_keymap_t` instances.

### Special key constants

`PS2_ENTER`, `PS2_TAB`, `PS2_ESC`, `PS2_BACKSPACE`, `PS2_DELETE`, `PS2_INSERT`,
`PS2_HOME`, `PS2_END`, `PS2_PAGEUP`, `PS2_PAGEDOWN`, `PS2_UPARROW`,
`PS2_DOWNARROW`, `PS2_LEFTARROW`, `PS2_RIGHTARROW`, `PS2_F1`..`PS2_F12`,
`PS2_SCROLL`.

## Notes on the GPIO ISR service

`ps2keyboard_begin()` calls `gpio_install_isr_service(ESP_INTR_FLAG_IRAM)`. If
your application has already installed the service with different flags this
call returns `ESP_ERR_INVALID_STATE`, which is treated as success - the
existing service is reused.

## Version History

- **v3.1.0** (May 2026) — Native ESP-IDF port (this branch)
- **v3.0.0** (January 2026) — ESP32 / ESP8266 (Arduino) support, Lock-key LEDs
- **v2.x** — Multi-platform Arduino library (Uno, Mega, Due, Teensy)

## Credits

Original library by [PJRC](http://www.pjrc.com/teensy/td_libs_PS2Keyboard.html).

- Christian Weichel — original author
- Paul Stoffregen — major rewrite
- L. Abraham Smith — Arduino 13 modifications
- Cuningan — flexible pin assignment
- Lahiru Nirmal — ESP32 / ESP8266 (Arduino) port and ESP-IDF port

## License

GNU Lesser General Public License v2.1
([LGPL-2.1](https://www.gnu.org/licenses/lgpl-2.1.html)).
