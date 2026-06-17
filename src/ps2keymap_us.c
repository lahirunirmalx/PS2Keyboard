/*
  ps2keymap_us.c - US keymap for the PS/2 keyboard ESP-IDF component.

  Scan code references:
    http://www.quadibloc.com/comp/scan.htm
    http://www.computer-engineering.org/ps2keyboard/scancodes2.html
*/

#include "ps2keyboard.h"

const ps2_keymap_t ps2_keymap_us = {
    /* without shift */
    {
        0, PS2_F9, 0, PS2_F5, PS2_F3, PS2_F1, PS2_F2, PS2_F12,
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
        0 /*CapsLock*/, 0 /*Rshift*/, PS2_ENTER, ']', 0, '\\', 0, 0,
        0, 0, 0, 0, 0, 0, PS2_BACKSPACE, 0,
        0, '1', 0, '4', '7', 0, 0, 0,
        '0', '.', '2', '5', '6', '8', PS2_ESC, 0 /*NumLock*/,
        PS2_F11, '+', '3', '-', '*', '9', PS2_SCROLL, 0,
        0, 0, 0, PS2_F7
    },
    /* with shift */
    {
        0, PS2_F9, 0, PS2_F5, PS2_F3, PS2_F1, PS2_F2, PS2_F12,
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
        0 /*CapsLock*/, 0 /*Rshift*/, PS2_ENTER, '}', 0, '|', 0, 0,
        0, 0, 0, 0, 0, 0, PS2_BACKSPACE, 0,
        0, '1', 0, '4', '7', 0, 0, 0,
        '0', '.', '2', '5', '6', '8', PS2_ESC, 0 /*NumLock*/,
        PS2_F11, '+', '3', '-', '*', '9', PS2_SCROLL, 0,
        0, 0, 0, PS2_F7
    },
    0, /* uses_altgr */
    { 0 },
};
