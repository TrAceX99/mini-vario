#include "baro.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "bmp3.h"

#define I2C_ADDRESS BMP3_ADDR_I2C_SEC

static const char *TAG = "BARO";

static struct bmp3_dev dev;
static i2c_master_bus_handle_t i2c_bus;
static i2c_master_dev_handle_t i2c_dev;

/*!
 * Delay function
 */
static void bmp3_user_delay_us(uint32_t period, void *intf_ptr)
{
    /* Wait for a period amount of microseconds. */
    vTaskDelay(pdMS_TO_TICKS(period / 1000));
}
/*!
 * I2C read function
 */
static BMP3_INTF_RET_TYPE bmp3_user_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    esp_err_t ret;
    i2c_master_dev_handle_t dev_handle = (i2c_master_dev_handle_t)intf_ptr;

    ret = i2c_master_transmit_receive(dev_handle, &reg_addr, 1, reg_data, len, 100);
    if (ret != ESP_OK)
    {
        return -1; // BMP3_E_COM_FAIL alternative
    }

    return 0; // BMP3_OK
}

/*!
 * I2C write function
 */
static BMP3_INTF_RET_TYPE bmp3_user_i2c_write(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    esp_err_t ret;
    i2c_master_dev_handle_t dev_handle = (i2c_master_dev_handle_t)intf_ptr;
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

    ret = i2c_master_multi_buffer_transmit(dev_handle, buffer_info, 2, 100);
    if (ret != ESP_OK)
    {
        return -1; // BMP3_E_COM_FAIL alternative
    }

    return 0; // BMP3_OK
}

static void baro_i2c_init(void)
{
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
}

void baro_init(void)
{
    int8_t res;

    baro_i2c_init();

    dev.intf = BMP3_I2C_INTF;
    dev.read = bmp3_user_i2c_read;
    dev.write = bmp3_user_i2c_write;
    dev.delay_us = bmp3_user_delay_us;
    dev.intf_ptr = i2c_dev;

    res = bmp3_init(&dev);
    if (unlikely(res != BMP3_OK))
    {
        ESP_LOGE(TAG, "bmp3_init failed (%d)", res);
        abort();
    }
    struct bmp3_settings settings = {0};

    settings.press_en = BMP3_ENABLE;
    settings.temp_en = BMP3_ENABLE;

    settings.odr_filter.press_os = BMP3_OVERSAMPLING_8X;
    settings.odr_filter.temp_os = BMP3_NO_OVERSAMPLING;
    settings.odr_filter.odr = BMP3_ODR_50_HZ;
    settings.odr_filter.iir_filter = BMP3_IIR_FILTER_COEFF_3;

    uint32_t settings_sel = BMP3_SEL_PRESS_EN | BMP3_SEL_TEMP_EN | BMP3_SEL_PRESS_OS | BMP3_SEL_TEMP_OS | BMP3_SEL_ODR | BMP3_SEL_IIR_FILTER;
    res = bmp3_set_sensor_settings(settings_sel, &settings, &dev);
    if (unlikely(res != BMP3_OK))
    {
        ESP_LOGE(TAG, "bmp3_set_sensor_settings failed (%d)", res);
        abort();
    }

    settings.op_mode = BMP3_MODE_NORMAL;
    res = bmp3_set_op_mode(&settings, &dev);
    if (unlikely(res != BMP3_OK))
    {
        ESP_LOGE(TAG, "bmp3_set_op_mode failed (%d)", res);
        abort();
    }
}

void baro_read(double *pressure, double *temperature)
{
    struct bmp3_data data = {0};
    int8_t res;

    res = bmp3_get_sensor_data(BMP3_PRESS_TEMP, &data, &dev);
    if (unlikely(res != BMP3_OK))
    {
        ESP_LOGE(TAG, "bmp3_get_sensor_data failed (%d)", res);
    }

    *pressure = data.pressure;
    *temperature = data.temperature;
}