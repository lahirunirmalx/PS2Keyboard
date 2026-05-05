/*
  PS2Keyboard Library - Simple Test Example
  
  This example demonstrates basic PS/2 keyboard input.
  
  Wiring:
    PS/2 Data  -> DataPin
    PS/2 Clock -> IRQpin
    PS/2 VCC   -> 5V
    PS/2 GND   -> GND

  Valid IRQ pins by board:
    Arduino Uno:    2, 3
    Arduino Mega:   2, 3, 18, 19, 20, 21
    Arduino Due:    All pins (except 13)
    Teensy 3.x/4.x: All digital pins
    ESP32/ESP8266:  All GPIO pins

  More info: https://github.com/lahirunirmalx/PS2Keyboard
  
  License: LGPL v2.1
*/

#include <PS2Keyboard.h>

const int DataPin = 8;
const int IRQpin = 5;

PS2Keyboard keyboard;

void setup() {
  delay(1000);
  keyboard.begin(DataPin, IRQpin);
  Serial.begin(115200);
  Serial.println("PS/2 Keyboard Test - Type to begin:");
}

void loop() {
  if (keyboard.available()) {
    
    // read the next key
    char c = keyboard.read();
    
    // check for some of the special keys
    if (c == PS2_ENTER) {
      Serial.println();
    } else if (c == PS2_TAB) {
      Serial.print("[Tab]");
    } else if (c == PS2_ESC) {
      Serial.print("[ESC]");
    } else if (c == PS2_PAGEDOWN) {
      Serial.print("[PgDn]");
    } else if (c == PS2_PAGEUP) {
      Serial.print("[PgUp]");
    } else if (c == PS2_LEFTARROW) {
      Serial.print("[Left]");
    } else if (c == PS2_RIGHTARROW) {
      Serial.print("[Right]");
    } else if (c == PS2_UPARROW) {
      Serial.print("[Up]");
    } else if (c == PS2_DOWNARROW) {
      Serial.print("[Down]");
    } else if (c == PS2_DELETE) {
      Serial.print("[Del]");
    } else {
      
      // otherwise, just print all normal characters
      Serial.print(c);
    }
  }
}
