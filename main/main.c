/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <inttypes.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "nvs_flash.h"
/* BLE */
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "console/console.h"
#include "services/gap/ble_svc_gap.h"

#include "bmp3.h"

#define ITERATION UINT8_C(100)
#define I2C_ADDRESS BMP3_ADDR_I2C_SEC
#define USEIIC 1

static const char *tag = "VARIO";
static const char *device_name = "MiniVario";

i2c_master_bus_handle_t i2c_bus;
i2c_master_dev_handle_t i2c_dev;

static uint8_t ble_prox_prph_addr_type;

/*!
 * Delay function
 */
void bmp3_user_delay_us(uint32_t period, void *intf_ptr)
{
    /* Wait for a period amount of microseconds. */
    vTaskDelay(pdMS_TO_TICKS(period / 1000));
}
/*!
 * I2C read function
 */
BMP3_INTF_RET_TYPE bmp3_user_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    esp_err_t ret;

    ret = i2c_master_transmit_receive(i2c_dev, &reg_addr, 1, reg_data, len, 100);
    if (ret != ESP_OK)
    {
        return -1;
    }

    return 0;
}

/*!
 * I2C write function
 */
BMP3_INTF_RET_TYPE bmp3_user_i2c_write(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    esp_err_t ret;
    i2c_master_transmit_multi_buffer_info_t buffer_info[2] = {
        {
            .write_buffer = &reg_addr,
            .buffer_size = 1,
        },
        {
            .write_buffer = reg_data,
            .buffer_size = len,
        },
    };

    ret = i2c_master_multi_buffer_transmit(i2c_dev, buffer_info, 2, 100);
    if (ret != ESP_OK)
    {
        return -1;
    }

    return 0;
}

void bmp3_check_rslt(const char api_name[], int8_t rslt)
{
    static const char *TAG = "BMP3";

    switch (rslt)
    {
    case BMP3_OK:
        /* Do nothing */
        break;
    case BMP3_E_NULL_PTR:
        ESP_LOGE(TAG, "API: %s Error: Null pointer; %d", api_name, rslt);
        break;
    case BMP3_E_COMM_FAIL:
        ESP_LOGE(TAG, "API: %s Error: Communication failure; %d", api_name, rslt);
        break;
    case BMP3_E_INVALID_LEN:
        ESP_LOGE(TAG, "API: %s Error: Incorrect length parameter; %d", api_name, rslt);
        break;
    case BMP3_E_DEV_NOT_FOUND:
        ESP_LOGE(TAG, "API: %s Error: Device not found; %d", api_name, rslt);
        break;
    case BMP3_E_CONFIGURATION_ERR:
        ESP_LOGE(TAG, "API: %s Error: Configuration Error; %d", api_name, rslt);
        break;
    case BMP3_W_SENSOR_NOT_ENABLED:
        ESP_LOGW(TAG, "API: %s Warning: Sensor not enabled; %d", api_name, rslt);
        break;
    case BMP3_W_INVALID_FIFO_REQ_FRAME_CNT:
        ESP_LOGW(TAG, "API: %s Warning: Fifo watermark level is not in limit; %d", api_name, rslt);
        break;
    default:
        ESP_LOGE(TAG, "API: %s Error: Unknown error code; %d", api_name, rslt);
        break;
    }
}

BMP3_INTF_RET_TYPE bmp3_interface_init(struct bmp3_dev *bmp3)
{
    int8_t rslt = BMP3_OK;

    if (bmp3 != NULL)
    {
        bmp3->intf = BMP3_I2C_INTF;
        bmp3->read = bmp3_user_i2c_read;
        bmp3->write = bmp3_user_i2c_write;
        bmp3->delay_us = bmp3_user_delay_us;
        bmp3->intf_ptr = &i2c_dev;
    }
    else
    {
        rslt = BMP3_E_NULL_PTR;
    }

    return rslt;
}

static uint8_t calculate_LK8EX1_checksum(const char *szNMEA)
{
    const char *sz = &szNMEA[1]; // skip leading '$'
    uint8_t cksum = 0;
    while ((*sz) != 0 && (*sz != '*'))
    {
        cksum ^= (uint8_t)*sz;
        sz++;
    }
    return cksum;
}

static void format_LK8EX1_string(char *buf, uint64_t pressure, int64_t temp, float battery_v)
{
    uint32_t final_pressure = (pressure + 50) / 100;
    int32_t final_temp = (temp + 50) / 100;
    sprintf(buf, "$LK8EX1,%lu,99999,9999,%ld,%.1f,*", final_pressure, final_temp, battery_v);
    uint8_t cksum = calculate_LK8EX1_checksum(buf);
    char chksum[5];
    sprintf(chksum, "%02X\r\n", cksum);
    strcat(buf, chksum);
}

static void
ble_prox_prph_advertise(void)
{
    struct ble_gap_adv_params adv_params;
    struct ble_hs_adv_fields fields;
    int rc;

    /*
     *  Set the advertisement data included in our advertisements:
     *     o Flags (indicates advertisement type and other general info)
     *     o Advertising tx power
     *     o Device name
     */
    memset(&fields, 0, sizeof(fields));

    /*
     * Advertise two flags:
     *      o Discoverability in forthcoming advertisement (general)
     *      o BLE-only (BR/EDR unsupported)
     */
    fields.flags = BLE_HS_ADV_F_DISC_GEN |
                   BLE_HS_ADV_F_BREDR_UNSUP;

    /*
     * Indicate that the TX power level field should be included; have the
     * stack fill this value automatically.  This is done by assigning the
     * special value BLE_HS_ADV_TX_PWR_LVL_AUTO.
     */
    fields.tx_pwr_lvl_is_present = 1;
    fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;

    fields.name = (uint8_t *)device_name;
    fields.name_len = strlen(device_name);
    fields.name_is_complete = 1;

    fields.uuids16 = (ble_uuid16_t[]){
        BLE_UUID16_INIT(BLE_SVC_LINK_LOSS_UUID16)};
    fields.num_uuids16 = 1;
    fields.uuids16_is_complete = 1;

    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0)
    {
        MODLOG_DFLT(ERROR, "error setting advertisement data; rc=%d\n", rc);
        return;
    }

    /* Begin advertising */
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    rc = ble_gap_adv_start(ble_prox_prph_addr_type, NULL, BLE_HS_FOREVER,
                           &adv_params, ble_prox_prph_gap_event, NULL);
    if (rc != 0)
    {
        MODLOG_DFLT(ERROR, "error enabling advertisement; rc=%d\n", rc);
        return;
    }
}

static void
ble_prox_prph_on_sync(void)
{
    int rc;

    rc = ble_hs_id_infer_auto(0, &ble_prox_prph_addr_type);
    assert(rc == 0);

    uint8_t addr_val[6] = {0};
    rc = ble_hs_id_copy_addr(ble_prox_prph_addr_type, addr_val, NULL);

    MODLOG_DFLT(INFO, "Device Address: ");
    print_addr(addr_val);
    MODLOG_DFLT(INFO, "\n");

    ble_prox_prph_advertise();
}

static void
ble_prox_prph_on_reset(int reason)
{
    MODLOG_DFLT(ERROR, "Resetting state; reason=%d\n", reason);
}

void ble_prox_prph_host_task(void *param)
{
    ESP_LOGI("MY", "BLE Host Task Started");
    /* This function will return only when nimble_port_stop() is executed */
    nimble_port_run();

    nimble_port_freertos_deinit();
}

int8_t rslt;
uint16_t settings_sel;
struct bmp3_dev dev;
struct bmp3_data data = {0};
struct bmp3_settings settings = {0};
struct bmp3_status status = {{0}};

void app_main(void)
{
    /* Interface reference is given as a parameter
     *         For I2C : BMP3_I2C_INTF
     *         For SPI : BMP3_SPI_INTF
     */

    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = 6,
        .scl_io_num = 7,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &i2c_bus));

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = BMP3_ADDR_I2C_SEC,
        .scl_speed_hz = 400000,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus, &dev_config, &i2c_dev));

    bmp3_interface_init(&dev);
    rslt = bmp3_init(&dev);
    bmp3_check_rslt("bmp3_init", rslt);

    settings.press_en = BMP3_ENABLE;
    settings.temp_en = BMP3_ENABLE;

    settings.odr_filter.press_os = BMP3_OVERSAMPLING_8X;
    settings.odr_filter.temp_os = BMP3_NO_OVERSAMPLING;
    settings.odr_filter.odr = BMP3_ODR_50_HZ;
    settings.odr_filter.iir_filter = BMP3_IIR_FILTER_COEFF_7;

    settings_sel = BMP3_SEL_PRESS_EN | BMP3_SEL_TEMP_EN | BMP3_SEL_PRESS_OS | BMP3_SEL_TEMP_OS | BMP3_SEL_ODR | BMP3_SEL_IIR_FILTER;

    rslt = bmp3_set_sensor_settings(settings_sel, &settings, &dev);
    bmp3_check_rslt("bmp3_set_sensor_settings", rslt);

    settings.op_mode = BMP3_MODE_NORMAL;
    rslt = bmp3_set_op_mode(&settings, &dev);
    bmp3_check_rslt("bmp3_set_op_mode", rslt);

    /* Initialize NVS — it is used to store PHY calibration data */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ret = nimble_port_init();
    if (ret != ESP_OK)
    {
        MODLOG_DFLT(ERROR, "Failed to init nimble %d \n", ret);
        return;
    }
    ble_svc_gap_device_name_set(device_name);

    while (1)
    {
        rslt = bmp3_get_status(&status, &dev);
        bmp3_check_rslt("bmp3_get_status", rslt);

        /* Read temperature and pressure data iteratively based on data ready interrupt */
        if (rslt == BMP3_OK)
        {
            rslt = bmp3_get_sensor_data(BMP3_PRESS_TEMP, &data, &dev);
            bmp3_check_rslt("bmp3_get_sensor_data", rslt);

            char msg[40];
            format_LK8EX1_string(msg, data.pressure, data.temperature, 0.0);
            // sprintf(msg, "%llu", data.pressure);
            printf("%s", msg);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
