# PS/2 Keyboard Library with ESP32 Support

This library provides support for using a PS/2 keyboard as an input device, with additional enhancements to support ESP32, making it compatible with LoRa-based chat applications and improved code structure.

## Original Project

The original PS/2 Keyboard Library was created by PJRC and allows easy interfacing of a PS/2 keyboard with microcontrollers, such as the Teensy. This library makes it easy to use a keyboard for user input. More details on the original project can be found here:

- [PS2Keyboard Library](http://www.pjrc.com/teensy/td_libs_PS2Keyboard.html)

![Original Library](http://www.pjrc.com/teensy/td_libs_PS2Keyboard.jpg)

## New Additions and Enhancements

This fork includes additional features and improvements:

- **ESP32 Support**: Added support for the ESP32 platform, making it compatible with LoRa-based chat applications and other ESP32 projects.
- **Efficient Interrupts**: Simplified interrupt handling for easier and more efficient management.
- **Caps Lock with Status LED**: Added support for the Caps Lock key along with an LED status indicator.
- **Code Optimization**: Removed bulky code sections, refactored to a modern C++ style, and enhanced code readability.

## Getting Started

### Prerequisites

- **Platform**: This library is compatible with both the ESP32 and Teensy platforms.
- **Dependencies**: Ensure you have the required PS/2 keyboard and the correct wiring setup for your microcontroller.

### Installation

1. Clone this repository or download the library as a ZIP file.
2. Include the library in your project:
   - For Arduino IDE: Go to `Sketch` -> `Include Library` -> `Add .ZIP Library...` and select the downloaded file.
   - For PlatformIO: Add this library to your project’s `lib` folder.

The following library dependency is shared across both environments:

```ini
[common]
lib_deps = 
  https://github.com/lahirunirmalx/PS2Keyboard#dev-master
```

### Environment Configuration

#### ESP32 Development Board

This environment is set up for the ESP32 board, utilizing the Arduino framework with a monitor speed of 115200 baud.

```ini
[env:esp32dev]
platform = espressif32
board = esp32dev
framework = arduino
monitor_speed = 115200
lib_deps = ${common.lib_deps}
```

#### Arduino Uno Board

This environment is configured for the Arduino Uno board, also using the Arduino framework and a monitor speed of 115200 baud.

```ini
[env:uno]
platform = atmelavr
board = uno
framework = arduino  
monitor_speed = 115200
lib_deps = ${common.lib_deps}
```

### Usage

After installing the library, initialize the PS/2 keyboard with the appropriate pins for data and clock:

```cpp
#include <PS2Keyboard.h>

const int DataPin = 13;
const int IRQPin = 12;
PS2Keyboard keyboard;

void setup() {
    Serial.begin(9600);
    keyboard.begin(DataPin, IRQPin);
    Serial.println("PS/2 Keyboard ready for input...");
}

void loop() {
    if (keyboard.available()) {
        char c = keyboard.read();
        Serial.print(c);
    }
}
```

Refer to the library's examples for more detailed usage.

## Contributing

Contributions are welcome! Whether it's bug fixes, new features, or documentation improvements, feel free to open an issue or submit a pull request.

### Support the Original Developers

This project is built upon the great work done by the original authors at PJRC. Please consider supporting their projects and acknowledging their contributions.

## License

This project is licensed under the [MIT License](LICENSE).

---

### Acknowledgments

Thanks to PJRC for the original PS/2 Keyboard Library and all contributors to the ESP32 and LoRa community who make embedded development more accessible.

--- 