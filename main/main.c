/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_pm.h"

#include "bt.h"
#include "baro.h"
#include "vario.h"
#include "config.h"
#include "battery.h"

#define TAG "APP"

static uint8_t nmea_checksum(const char *nmea_str)
{
    const char *sz = &nmea_str[1]; // skip leading '$'
    uint8_t cksum = 0;
    while ((*sz) != 0 && (*sz != '*'))
    {
        cksum ^= (uint8_t)*sz;
        sz++;
    }
    return cksum;
}

static void format_LK8EX1_string(char *buf, double pressure, float climb, double temp, float battery)
{
    battery = battery * 100.0f + 1000.0f;
    sprintf(buf, "$LK8EX1,%.2f,99999,%.0f,%.1f,%.0f,*", pressure, climb * 100, temp, battery);
    uint8_t cksum = nmea_checksum(buf);
    char chksum[5];
    sprintf(chksum, "%02X\r\n", cksum);
    strcat(buf, chksum);
}

void app_main(void)
{
    if (conf_enable_bluetooth) {
        bt_init();
    }
    baro_init();
    vario_init();
    battery_init();

    esp_pm_config_t pm_config = {
        .max_freq_mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ,
        .min_freq_mhz = CONFIG_XTAL_FREQ,
        .light_sleep_enable = false,
    };
    esp_pm_configure(&pm_config);

    vario_data_t data;
    const TickType_t period = pdMS_TO_TICKS(200);
    TickType_t last = xTaskGetTickCount();
    while (1) {
        BaseType_t delayed = xTaskDelayUntil(&last, period);
        if (unlikely(delayed != pdTRUE)) {
            ESP_LOGW(TAG, "Task delayed");
        }
        if (vario_get(&data)) {
            if (!conf_send_vario) {
                data.vspeed_mps = 99.99f;
            }
            char msg[48];
            format_LK8EX1_string(msg, data.pressure_pa, data.vspeed_mps, data.temperature_c, battery_get());
            if (bt_is_connected()) {
                bt_nus_send(msg, strlen(msg));
            }
            if (conf_enable_uart) {
                printf("%s", msg);
                // esp_pm_dump_locks(stdout);
                // fflush(stdout);
            }
        }
    }
}
