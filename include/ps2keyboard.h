/*
  ps2keyboard.h - PS/2 keyboard driver for ESP-IDF
  Copyright (c) 2007 Free Software Foundation. All rights reserved.

  Originally written by Christian Weichel <info@32leaves.net>.
  Mostly rewritten by Paul Stoffregen <paul@pjrc.com>, June 2010.
  Modified for Arduino 13 by L. Abraham Smith <n3bah@microcompdesign.com>.
  Modified for easy interrupt pin assignment by Cuningan <cuninganreset@gmail.com>.
  Modified for ESP32/ESP8266 (Arduino) by Lahiru <lahirunirmalx@gmail.com>.
  Ported to native ESP-IDF by Lahiru <lahirunirmalx@gmail.com>, 2026.

  This library is free software; you can redistribute it and/or modify it
  under the terms of the GNU Lesser General Public License as published by
  the Free Software Foundation; either version 2.1 of the License, or
  (at your option) any later version.
*/

#ifndef PS2KEYBOARD_H
#define PS2KEYBOARD_H

#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Special key codes returned by ps2keyboard_read(). Set to 0 to ignore. */
#define PS2_TAB             9
#define PS2_ENTER           13
#define PS2_BACKSPACE       127
#define PS2_ESC             27
#define PS2_INSERT          0
#define PS2_DELETE          127
#define PS2_HOME            0
#define PS2_END             0
#define PS2_PAGEUP          25
#define PS2_PAGEDOWN        26
#define PS2_UPARROW         11
#define PS2_LEFTARROW       8
#define PS2_DOWNARROW       10
#define PS2_RIGHTARROW      21
#define PS2_F1              0
#define PS2_F2              0
#define PS2_F3              0
#define PS2_F4              0
#define PS2_F5              0
#define PS2_F6              0
#define PS2_F7              0
#define PS2_F8              0
#define PS2_F9              0
#define PS2_F10             0
#define PS2_F11             0
#define PS2_F12             0
#define PS2_SCROLL          0
#define PS2_EURO_SIGN       0

/* ISO-8859-1 (Latin-1) symbols, available via shift / AltGr in some keymaps. */
#define PS2_INVERTED_EXCLAMATION    161
#define PS2_CENT_SIGN               162
#define PS2_POUND_SIGN              163
#define PS2_CURRENCY_SIGN           164
#define PS2_YEN_SIGN                165
#define PS2_BROKEN_BAR              166
#define PS2_SECTION_SIGN            167
#define PS2_DIAERESIS               168
#define PS2_COPYRIGHT_SIGN          169
#define PS2_FEMININE_ORDINAL        170
#define PS2_LEFT_DOUBLE_ANGLE_QUOTE 171
#define PS2_NOT_SIGN                172
#define PS2_HYPHEN                  173
#define PS2_REGISTERED_SIGN         174
#define PS2_MACRON                  175
#define PS2_DEGREE_SIGN             176
#define PS2_PLUS_MINUS_SIGN         177
#define PS2_SUPERSCRIPT_TWO         178
#define PS2_SUPERSCRIPT_THREE       179
#define PS2_ACUTE_ACCENT            180
#define PS2_MICRO_SIGN              181
#define PS2_PILCROW_SIGN            182
#define PS2_MIDDLE_DOT              183
#define PS2_CEDILLA                 184
#define PS2_SUPERSCRIPT_ONE         185
#define PS2_MASCULINE_ORDINAL       186
#define PS2_RIGHT_DOUBLE_ANGLE_QUOTE 187
#define PS2_FRACTION_ONE_QUARTER    188
#define PS2_FRACTION_ONE_HALF       189
#define PS2_FRACTION_THREE_QUARTERS 190
#define PS2_INVERTED_QUESTION_MARK  191
#define PS2_A_GRAVE                 192
#define PS2_A_ACUTE                 193
#define PS2_A_CIRCUMFLEX            194
#define PS2_A_TILDE                 195
#define PS2_A_DIAERESIS             196
#define PS2_A_RING_ABOVE            197
#define PS2_AE                      198
#define PS2_C_CEDILLA               199
#define PS2_E_GRAVE                 200
#define PS2_E_ACUTE                 201
#define PS2_E_CIRCUMFLEX            202
#define PS2_E_DIAERESIS             203
#define PS2_I_GRAVE                 204
#define PS2_I_ACUTE                 205
#define PS2_I_CIRCUMFLEX            206
#define PS2_I_DIAERESIS             207
#define PS2_ETH                     208
#define PS2_N_TILDE                 209
#define PS2_O_GRAVE                 210
#define PS2_O_ACUTE                 211
#define PS2_O_CIRCUMFLEX            212
#define PS2_O_TILDE                 213
#define PS2_O_DIAERESIS             214
#define PS2_MULTIPLICATION          215
#define PS2_O_STROKE                216
#define PS2_U_GRAVE                 217
#define PS2_U_ACUTE                 218
#define PS2_U_CIRCUMFLEX            219
#define PS2_U_DIAERESIS             220
#define PS2_Y_ACUTE                 221
#define PS2_THORN                   222
#define PS2_SHARP_S                 223
#define PS2_a_GRAVE                 224
#define PS2_a_ACUTE                 225
#define PS2_a_CIRCUMFLEX            226
#define PS2_a_TILDE                 227
#define PS2_a_DIAERESIS             228
#define PS2_a_RING_ABOVE            229
#define PS2_ae                      230
#define PS2_c_CEDILLA               231
#define PS2_e_GRAVE                 232
#define PS2_e_ACUTE                 233
#define PS2_e_CIRCUMFLEX            234
#define PS2_e_DIAERESIS             235
#define PS2_i_GRAVE                 236
#define PS2_i_ACUTE                 237
#define PS2_i_CIRCUMFLEX            238
#define PS2_i_DIAERESIS             239
#define PS2_eth                     240
#define PS2_n_TILDE                 241
#define PS2_o_GRAVE                 242
#define PS2_o_ACUTE                 243
#define PS2_o_CIRCUMFLEX            244
#define PS2_o_TILDE                 245
#define PS2_o_DIAERESIS             246
#define PS2_DIVISION                247
#define PS2_o_STROKE                248
#define PS2_u_GRAVE                 249
#define PS2_u_ACUTE                 250
#define PS2_u_CIRCUMFLEX            251
#define PS2_u_DIAERESIS             252
#define PS2_y_ACUTE                 253
#define PS2_thorn                   254
#define PS2_y_DIAERESIS             255

#define PS2_KEYMAP_SIZE 136

typedef struct {
    uint8_t  noshift[PS2_KEYMAP_SIZE];
    uint8_t  shift[PS2_KEYMAP_SIZE];
    uint32_t uses_altgr;
    uint8_t  altgr[PS2_KEYMAP_SIZE];
} ps2_keymap_t;

/* US keyboard layout (default). */
extern const ps2_keymap_t ps2_keymap_us;

/**
 * @brief Initialise the PS/2 keyboard driver.
 *
 * Configures the data and clock GPIO pins, installs the GPIO ISR service
 * (if not already installed) and attaches the falling-edge interrupt on the
 * clock pin.
 *
 * @param data_pin   GPIO connected to the PS/2 DATA line.
 * @param clock_pin  GPIO connected to the PS/2 CLOCK line (interrupt source).
 * @param keymap     Keymap to use (e.g. &ps2_keymap_us). Pass NULL for default.
 *
 * @return ESP_OK on success, otherwise an esp_err_t code from the GPIO driver.
 */
esp_err_t ps2keyboard_begin(gpio_num_t data_pin,
                            gpio_num_t clock_pin,
                            const ps2_keymap_t *keymap);

/**
 * @brief Release ISR resources and reset internal state.
 */
void ps2keyboard_end(void);

/**
 * @brief Returns true if at least one decoded character is ready.
 */
bool ps2keyboard_available(void);

/**
 * @brief Discard any buffered character without returning it.
 */
void ps2keyboard_clear(void);

/**
 * @brief Pop one raw PS/2 scan code from the receive buffer.
 *
 * @return Scan code (0x00..0xFF), or 0 if the buffer is empty.
 */
uint8_t ps2keyboard_read_scancode(void);

/**
 * @brief Read the next decoded character, expanding non-ASCII to UTF-8.
 *
 * @return The byte read (0x00..0xFF), or -1 if no character is available.
 */
int ps2keyboard_read(void);

/**
 * @brief Read the next decoded character as a single Unicode code point.
 *
 * @return The code point (ISO-8859-1 range), or -1 if none available.
 */
int ps2keyboard_read_unicode(void);

#ifdef __cplusplus
}
#endif

#endif /* PS2KEYBOARD_H */
