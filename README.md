# PS2Keyboard Library

[![License: LGPL v2.1](https://img.shields.io/badge/License-LGPL%20v2.1-blue.svg)](https://www.gnu.org/licenses/lgpl-2.1)
[![Version](https://img.shields.io/badge/version-3.0.0-green.svg)](https://github.com/lahirunirmalx/PS2Keyboard)

A PS/2 keyboard library for Arduino, Teensy, ESP32, and ESP8266 platforms.

## Features

- **Multi-platform support**: Arduino (Uno, Mega, Due, Leonardo), Teensy, ESP32, ESP8266
- **Interrupt-driven**: Efficient interrupt-based input handling
- **Lock key support**: Caps Lock, Num Lock, and Scroll Lock with LED indicators
- **Easy to use**: Simple API with `begin()`, `available()`, and `read()` methods
- **US keyboard layout**: Standard US QWERTY layout included

## Installation

### Arduino IDE

1. Download this repository as a ZIP file
2. In Arduino IDE: `Sketch` → `Include Library` → `Add .ZIP Library...`
3. Select the downloaded ZIP file

### PlatformIO

Add the following to your `platformio.ini`:

```ini
lib_deps =
  https://github.com/lahirunirmalx/PS2Keyboard
```

## Wiring

Connect your PS/2 keyboard to your microcontroller:

| PS/2 Pin | Signal | Connect To |
|----------|--------|------------|
| 1        | Data   | Data Pin (configurable) |
| 3        | GND    | GND |
| 4        | VCC    | 5V |
| 5        | Clock  | IRQ Pin (configurable) |

> **Note**: PS/2 keyboards require 5V power. Some boards may need level shifters for the data lines.

## Quick Start

```cpp
#include <PS2Keyboard.h>

const int DataPin = 8;
const int IRQpin = 5;

PS2Keyboard keyboard;

void setup() {
  Serial.begin(115200);
  keyboard.begin(DataPin, IRQpin);
  Serial.println("PS/2 Keyboard Ready");
}

void loop() {
  if (keyboard.available()) {
    char c = keyboard.read();
    
    if (c == PS2_ENTER) {
      Serial.println();
    } else if (c == PS2_TAB) {
      Serial.print("[Tab]");
    } else if (c == PS2_ESC) {
      Serial.print("[ESC]");
    } else if (c == PS2_BACKSPACE) {
      Serial.print("[Backspace]");
    } else {
      Serial.print(c);
    }
  }
}
```

## Valid IRQ Pins

| Board | Valid IRQ Pins |
|-------|----------------|
| Arduino Uno | 2, 3 |
| Arduino Mega | 2, 3, 18, 19, 20, 21 |
| Arduino Due | All pins (except 13) |
| Arduino Leonardo | 0, 1, 2, 3 |
| Teensy 3.x/4.x | All digital pins |
| Teensy 2.0 | 5, 6, 7, 8 |
| ESP32 | All GPIO pins |
| ESP8266 | All GPIO pins |

## API Reference

### Methods

| Method | Description |
|--------|-------------|
| `begin(dataPin, irqPin)` | Initialize the keyboard with data and clock pins |
| `begin(dataPin, irqPin, keymap)` | Initialize with a custom keymap |
| `available()` | Returns `true` if a key is available to read |
| `read()` | Returns the next character (UTF-8 encoded) |
| `readUnicode()` | Returns the next character as Unicode |
| `readScanCode()` | Returns the raw PS/2 scan code |
| `clear()` | Clears the keyboard buffer |

### Special Key Constants

```cpp
PS2_ENTER       // Enter key
PS2_TAB         // Tab key
PS2_ESC         // Escape key
PS2_BACKSPACE   // Backspace key
PS2_DELETE      // Delete key
PS2_INSERT      // Insert key
PS2_HOME        // Home key
PS2_END         // End key
PS2_PAGEUP      // Page Up key
PS2_PAGEDOWN    // Page Down key
PS2_UPARROW     // Up arrow
PS2_DOWNARROW   // Down arrow
PS2_LEFTARROW   // Left arrow
PS2_RIGHTARROW  // Right arrow
PS2_F1 - PS2_F12 // Function keys
```

## Platform-Specific Configuration

### ESP32

```ini
[env:esp32dev]
platform = espressif32
board = esp32dev
framework = arduino
monitor_speed = 115200
lib_deps =
  https://github.com/lahirunirmalx/PS2Keyboard
```

### Arduino Uno

```ini
[env:uno]
platform = atmelavr
board = uno
framework = arduino
monitor_speed = 115200
lib_deps =
  https://github.com/lahirunirmalx/PS2Keyboard
```

## Examples

- **Simple_Test**: Basic keyboard input example
- **TypeToDisplay**: Display keyboard input on an LCD

## Version History

- **v3.0.0** (January 2026)
  - ESP32 and ESP8266 platform support
  - Improved interrupt handling
  - Caps Lock, Num Lock, Scroll Lock LED support
  - Code cleanup and optimization

- **v2.4** (March 2013)
  - Teensy 3.0, Arduino Due, Leonardo support

- **v2.0** (June 2010)
  - Buffering, shift key support, indexed lookups

## Credits

- Original library by [PJRC](http://www.pjrc.com/teensy/td_libs_PS2Keyboard.html)
- Christian Weichel - Original author
- Paul Stoffregen - Major rewrite
- L. Abraham Smith - Arduino 13 modifications
- Cuningan - Flexible pin assignment
- Lahiru - ESP32/ESP8266 support and maintenance

## License

This library is licensed under the [GNU Lesser General Public License v2.1](https://www.gnu.org/licenses/lgpl-2.1.html).

## Contributing

Contributions are welcome! Please open an issue or submit a pull request.

## Support

- [GitHub Issues](https://github.com/lahirunirmalx/PS2Keyboard/issues)
- [Arduino Forum](https://forum.arduino.cc)
- [ESP32 Forum](https://www.esp32.com)
