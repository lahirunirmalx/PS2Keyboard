/*
  PS/2 Keyboard ESP-IDF Example - simple_test

  Wiring:
    PS/2 DATA  -> CONFIG_PS2_DATA_PIN  (default GPIO 18)
    PS/2 CLOCK -> CONFIG_PS2_CLOCK_PIN (default GPIO 19)
    PS/2 VCC   -> 5V (level-shift to 3.3V if your board cannot tolerate 5V on GPIOs)
    PS/2 GND   -> GND

  License: LGPL v2.1
*/

#include <stdio.h>

#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "ps2keyboard.h"

#define PS2_DATA_PIN   GPIO_NUM_18
#define PS2_CLOCK_PIN  GPIO_NUM_19

static const char *TAG = "ps2_example";

void app_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(1000));

    ESP_ERROR_CHECK(ps2keyboard_begin(PS2_DATA_PIN, PS2_CLOCK_PIN, NULL));
    ESP_LOGI(TAG, "PS/2 keyboard ready - type to begin:");

    while (1) {
        if (ps2keyboard_available()) {
            int c = ps2keyboard_read();

            switch (c) {
                case PS2_ENTER:      printf("\n");      break;
                case PS2_TAB:        printf("[Tab]");   break;
                case PS2_ESC:        printf("[ESC]");   break;
                case PS2_PAGEDOWN:   printf("[PgDn]");  break;
                case PS2_PAGEUP:     printf("[PgUp]");  break;
                case PS2_LEFTARROW:  printf("[Left]");  break;
                case PS2_RIGHTARROW: printf("[Right]"); break;
                case PS2_UPARROW:    printf("[Up]");    break;
                case PS2_DOWNARROW:  printf("[Down]");  break;
                case PS2_DELETE:     printf("[Del]");   break;
                default:
                    if (c > 0) {
                        putchar(c);
                    }
                    break;
            }
            fflush(stdout);
        } else {
            vTaskDelay(pdMS_TO_TICKS(5));
        }
    }
}
