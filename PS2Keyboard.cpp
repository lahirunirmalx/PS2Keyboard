/*
  PS2Keyboard.cpp - PS2Keyboard library
  Copyright (c) 2007 Free Software Foundation.  All right reserved.
  Written by Christian Weichel <info@32leaves.net>

  ** Mostly rewritten Paul Stoffregen <paul@pjrc.com> 2010, 2011
  ** Modified for use beginning with Arduino 13 by L. Abraham Smith, <n3bah@microcompdesign.com> * 
  ** Modified for easy interrup pin assignement on method begin(datapin,irq_pin). Cuningan <cuninganreset@gmail.com> **
  ** Modified for ESP32 support **

  for more information you can read the original wiki in arduino.cc
  at http://www.arduino.cc/playground/Main/PS2Keyboard
  or http://www.pjrc.com/teensy/td_libs_PS2Keyboard.html

  Version 2.5 (January 2025)
  - Better Inturpts
  - Support for ESP32 

  Version 2.4 (March 2013)
  - Support Teensy 3.0, Arduino Due, Arduino Leonardo & other boards
  - French keyboard layout, David Chochoi, tchoyyfr at yahoo dot fr

  Version 2.3 (October 2011)
  - Minor bugs fixed

  Version 2.2 (August 2011)
  - Support non-US keyboards - thanks to Rainer Bruch for a German keyboard :)

  Version 2.1 (May 2011)
  - timeout to recover from misaligned input
  - compatibility with Arduino "new-extension" branch
  - TODO: send function, proposed by Scott Penrose, scooterda at me dot com

  Version 2.0 (June 2010)
  - Buffering added, many scan codes can be captured without data loss
    if your sketch is busy doing other work
  - Shift keys supported, completely rewritten scan code to ascii
  - Slow linear search replaced with fast indexed table lookups
  - Support for Teensy, Arduino Mega, and Sanguino added

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "PS2Keyboard.h"

#define BUFFER_SIZE 45
static volatile uint8_t buffer[BUFFER_SIZE];
static volatile uint8_t head, tail;
static uint8_t dataPin;
static uint8_t IRQPin;
static uint8_t CharBuffer=0;
static uint8_t UTF8next=0;
static const PS2Keymap_t *keymap=NULL;
static bool capsLockOn = false;
static bool numLockOn = false;
static bool scrollLockOn = false;
static byte ledState = 0;

#define LED_CONTROL 0xED

#ifdef ESP32
#define SUPPORT_IRAM_ATTR IRAM_ATTR
#else
#define SUPPORT_IRAM_ATTR  // Empty definition for non-ESP32 boards
#endif

// Parity check
// Return true for odd parity.
static bool parityCheck(uint16_t data)
{
   uint8_t count = 0; 
    for (int i = 0; i < 10; i++) {
        count += (data >> i) & 1;   
    } 
    return count % 2 != 0;
}
/*
code taken from https://karooza.net/how-to-interface-a-ps2-keyboard
parity was needed and the writing should have done on rising edeg not in falling 
*/
static void sendPS2Command(uint8_t cmdCode) {
    uint16_t cmd = cmdCode;  // Cast to 16-bit to add parity and stop bits

    // Add parity bit if not already set
    if (!parityCheck(cmd)) {
        cmd |= 0x0100;
    }

    // Add stop bit
    cmd |= 0x0200;

    // Set clock and data pins to output mode
    pinMode(IRQPin, OUTPUT);
    pinMode(dataPin, OUTPUT);

    // Start Request to Send by pulling clock low for 100us
    digitalWrite(IRQPin, LOW);
    unsigned long startMicros = micros();
    while (micros() - startMicros < 100);  // Wait for 100us

    // Pull data low to send the start bit
    digitalWrite(dataPin, LOW);
    startMicros = micros();
    while (micros() - startMicros < 20);   // Wait for 20us

    // Release clock line, allowing it to go high
    pinMode(IRQPin, INPUT_PULLUP);

    // Send 8 data bits, 1 parity bit, and 1 stop bit
    for (int i = 0; i < 10; i++) {
        // Wait for keyboard to take clock low (start of bit transmission)
        while (digitalRead(IRQPin) > 0);

        // Write the current bit to the data line
        digitalWrite(dataPin, cmd & 0x0001);
        cmd >>= 1;  // Shift to the next bit

        // Wait for the keyboard to take clock high, indicating bit has been sampled
        while (digitalRead(IRQPin) < 1);
    }

    // Release data line (high) for the stop bit
    pinMode(dataPin, INPUT_PULLUP);

    // Wait for keyboard to acknowledge by taking clock low
    while (digitalRead(IRQPin) > 0);

    // Wait for keyboard ACK by checking data line goes low
    while (digitalRead(dataPin) > 0);

    // Reset pins to input mode with pull-up resistors
#ifdef INPUT_PULLUP
    pinMode(IRQPin, INPUT_PULLUP);
    pinMode(dataPin, INPUT_PULLUP);
#else
    pinMode(IRQPin, INPUT);
    digitalWrite(IRQPin, HIGH);
    pinMode(dataPin, INPUT);
    digitalWrite(dataPin, HIGH);
#endif 
}

static void updateLEDs(byte command){
	
	//Serial.print(LED_CONTROL);
	sendPS2Command(LED_CONTROL);
   // Serial.print(command); 
   sendPS2Command(command);
   // Could not get this working 
}

// The ISR for the external interrupt
void SUPPORT_IRAM_ATTR ps2interrupt(void) {
  static uint8_t bitCount = 0;
  static uint8_t incoming = 0;
  static uint32_t prevMillis = 0;
  uint32_t nowMillis;
  uint8_t bitPos, val;

  val = digitalRead(dataPin);
  nowMillis = millis();
  if (nowMillis - prevMillis > 250) {
    bitCount = 0;
    incoming = 0;
  }
  prevMillis = nowMillis;
  bitPos = bitCount - 1;
  if (bitPos <= 7) {
    incoming |= (val << bitPos);
  }
  bitCount++;
  if (bitCount == 11) {
    uint8_t i = head + 1;
    if (i >= BUFFER_SIZE) i = 0;
    if (i != tail) {
      buffer[i] = incoming;
      head = i;
    }
    bitCount = 0;
    incoming = 0;
  }
}

static inline uint8_t getScanCode(void) {
  uint8_t c, i;

  i = tail;
  if (i == head) return 0;
  i++;
  if (i >= BUFFER_SIZE) i = 0;
  c = buffer[i];
  tail = i;
  return c;
}

// http://www.quadibloc.com/comp/scan.htm
// http://www.computer-engineering.org/ps2keyboard/scancodes2.html

// These arrays provide a simple key map, to turn scan codes into ISO8859
// output.  If a non-US keyboard is used, these may need to be modified
// for the desired output.
//

const PROGMEM PS2Keymap_t PS2Keymap_US = {
  // without shift
	{0, PS2_F9, 0, PS2_F5, PS2_F3, PS2_F1, PS2_F2, PS2_F12,
	0, PS2_F10, PS2_F8, PS2_F6, PS2_F4, PS2_TAB, '`', 0,
	0, 0 /*Lalt*/, 0 /*Lshift*/, 0, 0 /*Lctrl*/, 'q', '1', 0,
	0, 0, 'z', 's', 'a', 'w', '2', 0,
	0, 'c', 'x', 'd', 'e', '4', '3', 0,
	0, ' ', 'v', 'f', 't', 'r', '5', 0,
	0, 'n', 'b', 'h', 'g', 'y', '6', 0,
	0, 0, 'm', 'j', 'u', '7', '8', 0,
	0, ',', 'k', 'i', 'o', '0', '9', 0,
	0, '.', '/', 'l', ';', 'p', '-', 0,
	0, 0, '\'', 0, '[', '=', 0, 0,
	0 /*CapsLock*/, 0 /*Rshift*/, PS2_ENTER /*Enter*/, ']', 0, '\\', 0, 0,
	0, 0, 0, 0, 0, 0, PS2_BACKSPACE, 0,
	0, '1', 0, '4', '7', 0, 0, 0,
	'0', '.', '2', '5', '6', '8', PS2_ESC, 0 /*NumLock*/,
	PS2_F11, '+', '3', '-', '*', '9', PS2_SCROLL, 0,
	0, 0, 0, PS2_F7 },
  // with shift
	{0, PS2_F9, 0, PS2_F5, PS2_F3, PS2_F1, PS2_F2, PS2_F12,
	0, PS2_F10, PS2_F8, PS2_F6, PS2_F4, PS2_TAB, '~', 0,
	0, 0 /*Lalt*/, 0 /*Lshift*/, 0, 0 /*Lctrl*/, 'Q', '!', 0,
	0, 0, 'Z', 'S', 'A', 'W', '@', 0,
	0, 'C', 'X', 'D', 'E', '$', '#', 0,
	0, ' ', 'V', 'F', 'T', 'R', '%', 0,
	0, 'N', 'B', 'H', 'G', 'Y', '^', 0,
	0, 0, 'M', 'J', 'U', '&', '*', 0,
	0, '<', 'K', 'I', 'O', ')', '(', 0,
	0, '>', '?', 'L', ':', 'P', '_', 0,
	0, 0, '"', 0, '{', '+', 0, 0,
	0 /*CapsLock*/, 0 /*Rshift*/, PS2_ENTER /*Enter*/, '}', 0, '|', 0, 0,
	0, 0, 0, 0, 0, 0, PS2_BACKSPACE, 0,
	0, '1', 0, '4', '7', 0, 0, 0,
	'0', '.', '2', '5', '6', '8', PS2_ESC, 0 /*NumLock*/,
	PS2_F11, '+', '3', '-', '*', '9', PS2_SCROLL, 0,
	0, 0, 0, PS2_F7 },
	0
};

 // no hate but I have remove this 

#define BREAK     0x01
#define MODIFIER  0x02
#define SHIFT_L   0x04
#define SHIFT_R   0x08
#define ALTGR     0x10

#define SCROLL_LOCK 0x01 
#define NUM_LOCK    0x02 
#define CAPS_LOCK   0x04 

#define LED_CONTROL 0xED



static char get_iso8859_code(void) {
    static uint8_t state = 0;
    uint8_t scanCode;
    char character;

    while (1) {
        // Retrieve the scan code
        scanCode = getScanCode();
        if (!scanCode) return 0; // No scan code available

        // Handle special cases for scan codes
        if (scanCode == 0xF0) {
            state |= BREAK; // Set BREAK state
        } else if (scanCode == 0xE0) {
            state |= MODIFIER; // Set MODIFIER state for extended keys
        } else {
            // Handle key release (BREAK state)
            if (state & BREAK) {
                if (scanCode == 0x12) {
                    state &= ~SHIFT_L; // Release Left Shift
                } else if (scanCode == 0x59) {
                    state &= ~SHIFT_R; // Release Right Shift
                } else if (scanCode == 0x11 && (state & MODIFIER)) {
                    state &= ~ALTGR; // Release AltGr
                }
                // Reset states and continue to next scan code
                state &= ~(BREAK | MODIFIER);
                continue;
            }

            // Handle key press events for modifiers and locks
            if (scanCode == 0x12) {
                state |= SHIFT_L; // Left Shift pressed
                continue;
            } else if (scanCode == 0x59) {
                state |= SHIFT_R; // Right Shift pressed
                continue;
            } else if (scanCode == 0x11 && (state & MODIFIER)) {
                state |= ALTGR; // AltGr pressed
            } else if (scanCode == 0x58) {
                capsLockOn = !capsLockOn; // Toggle Caps Lock
                ledState = capsLockOn ? (ledState | CAPS_LOCK) : (ledState & ~CAPS_LOCK);
                updateLEDs(ledState);
            } else if (scanCode == 0x77) {
                numLockOn = !numLockOn; // Toggle Num Lock
                ledState = numLockOn ? (ledState | NUM_LOCK) : (ledState & ~NUM_LOCK);
				updateLEDs(ledState);
            } else if (scanCode == 0x7E) {
                scrollLockOn = !scrollLockOn; // Toggle Scroll Lock
                ledState = scrollLockOn ? (ledState | SCROLL_LOCK) : (ledState & ~SCROLL_LOCK);
				updateLEDs(ledState);
            }

            // Process scan codes based on the current state
            character = 0;
            if (state & MODIFIER) {
                // Handle special keys with modifier (like arrow keys)
                switch (scanCode) {
                    case 0x70: character = PS2_INSERT;      break;
                    case 0x6C: character = PS2_HOME;        break;
                    case 0x7D: character = PS2_PAGEUP;      break;
                    case 0x71: character = PS2_DELETE;      break;
                    case 0x69: character = PS2_END;         break;
                    case 0x7A: character = PS2_PAGEDOWN;    break;
                    case 0x75: character = PS2_UPARROW;     break;
                    case 0x6B: character = PS2_LEFTARROW;   break;
                    case 0x72: character = PS2_DOWNARROW;   break;
                    case 0x74: character = PS2_RIGHTARROW;  break;
                    case 0x4A: character = '/';             break;
                    case 0x5A: character = PS2_ENTER;       break;
                    default: break;
                }
            } else if ((state & ALTGR) && pgm_read_byte(keymap->uses_altgr)) {
                // Handle AltGr key combinations
                if (scanCode < PS2_KEYMAP_SIZE) {
                    character = pgm_read_byte(keymap->altgr + scanCode);
                }
            } else if (capsLockOn) {
                // Handle Caps Lock with or without Shift keys
                if (state & (SHIFT_L | SHIFT_R)) {
                    if (scanCode < PS2_KEYMAP_SIZE) {
                        character = pgm_read_byte(keymap->noshift + scanCode);
                    }
                } else {
                    if (scanCode < PS2_KEYMAP_SIZE) {
                        character = pgm_read_byte(keymap->shift + scanCode);
                    }
                }
            } else {
                // Handle regular Shift and non-Shift key states
                if (state & (SHIFT_L | SHIFT_R)) {
                    if (scanCode < PS2_KEYMAP_SIZE) {
                        character = pgm_read_byte(keymap->shift + scanCode);
                    }
                } else {
                    if (scanCode < PS2_KEYMAP_SIZE) {
                        character = pgm_read_byte(keymap->noshift + scanCode);
                    }
                }
            }

            // Reset the states for the next scan code processing
            state &= ~(BREAK | MODIFIER);

            // Return the character if one was generated
            if (character) return character;
        }
    }
}




bool PS2Keyboard::available() {
	if (CharBuffer || UTF8next) return true;
	CharBuffer = get_iso8859_code();
	if (CharBuffer) return true;
	return false;
}

void PS2Keyboard::clear() {
	CharBuffer = 0;
	UTF8next = 0;
}

uint8_t PS2Keyboard::readScanCode(void)
{
	return getScanCode();
}

int PS2Keyboard::read() {
	uint8_t result;

	result = UTF8next;
	if (result) {
		UTF8next = 0;
	} else {
		result = CharBuffer;
		if (result) {
			CharBuffer = 0;
		} else {
			result = get_iso8859_code();
		}
		if (result >= 128) {
			UTF8next = (result & 0x3F) | 0x80;
			result = ((result >> 6) & 0x1F) | 0xC0;
		}
	}
	if (!result) return -1;
	return result;
}

int PS2Keyboard::readUnicode() {
	int result;

	result = CharBuffer;
	if (!result) result = get_iso8859_code();
	if (!result) return -1;
	UTF8next = 0;
	CharBuffer = 0;
	return result;
}

PS2Keyboard::PS2Keyboard() {
  // nothing to do here, begin() does it all
}

void PS2Keyboard::begin(uint8_t data_pin, uint8_t irq_pin, const PS2Keymap_t &map) { 
  uint8_t irq_num = irq_pin;
  IRQPin= irq_pin;
  dataPin = data_pin;
  keymap = &map;

  // initialize the pins
#ifdef INPUT_PULLUP
  pinMode(irq_pin, INPUT_PULLUP);
  pinMode(data_pin, INPUT_PULLUP);
#else
  pinMode(irq_pin, INPUT);
  digitalWrite(irq_pin, HIGH);
  pinMode(data_pin, INPUT);
  digitalWrite(data_pin, HIGH);
#endif 
  head = 0;
  tail = 0;
  irq_num = digitalPinToInterrupt(irq_pin);
  if (irq_num < 255) {
    attachInterrupt(irq_num, ps2interrupt, FALLING);
  }
}


