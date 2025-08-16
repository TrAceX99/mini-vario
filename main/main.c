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
/* BLE */
#include "bt.h"

#include "baro.h"

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

static void format_LK8EX1_string(char *buf, double pressure, double temp, float battery_v)
{
    sprintf(buf, "$LK8EX1,%.0f,99999,9999,%.1f,%.1f,*", pressure, temp, battery_v);
    uint8_t cksum = nmea_checksum(buf);
    char chksum[5];
    sprintf(chksum, "%02X\r\n", cksum);
    strcat(buf, chksum);
}

void app_main(void)
{
    bt_init();
    while (1)
    {
        double pressure = 0.0;
        double temperature = 0.0;

        baro_read(&pressure, &temperature);

        char msg[40];
        format_LK8EX1_string(msg, pressure, temperature, 0.0);
        // sprintf(msg, "%llu", data.pressure);
        printf("%s", msg);
        if (bt_is_connected()) {
            bt_nus_send(msg, strlen(msg));
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
