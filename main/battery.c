// battery.c - Battery voltage measurement module
// Periodically samples battery voltage via ADC and provides filtered percentage.

#include "battery.h"
#include "config.h"
#include "baro.h"

#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_sleep.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#define BATTERY_TASK_NAME "battery"
#define BATTERY_TASK_STACK 4096
#define BATTERY_TASK_PRIO 3
#define BATTERY_SAMPLE_PERIOD_MS 2000
#define BATTERY_ADC_CHANNEL ADC_CHANNEL_4 /* GPIO4 on ESP32-C3 -> ADC1_CH4 */
#define BATTERY_ADC_ATTEN ADC_ATTEN_DB_12 /* ~2.4V full-scale typical */

// Voltage divider attenuation ratio (ADC sees Vbatt * R2/(R1+R2)).
#define BATTERY_DIVIDER_FACTOR 2.0f

// Multisampling count per periodic reading
#define BATTERY_MULTISAMPLE_COUNT 32

// Filtering constants
#define BATTERY_EXP_ALPHA 0.25f

static const char *TAG = "BAT";

static TaskHandle_t s_task = NULL;
static float s_voltage_v = 0.0f;  // filtered battery voltage
static float s_percentage = 0.0f; // 0..1 filtered percentage
static bool s_valid = false;
static bool s_inact_forced = false; // whether inactivity timeout forced to 3s

// ADC handles (oneshot + calibration)
static adc_oneshot_unit_handle_t s_adc_handle = NULL;
static adc_cali_handle_t s_cali_handle = NULL;
static bool s_cali_enabled = false;

// Battery voltage to percentage mapping (simple linear between min & max)
#ifndef BATTERY_VOLTAGE_MIN
#define BATTERY_VOLTAGE_MIN 3.3f
#endif
#ifndef BATTERY_VOLTAGE_MAX
#define BATTERY_VOLTAGE_MAX 4.18f
#endif

static float map_voltage_to_percent(float v)
{
    if (v <= BATTERY_VOLTAGE_MIN)
        return 0.0f;
    if (v >= BATTERY_VOLTAGE_MAX)
        return 1.0f;
    return (v - BATTERY_VOLTAGE_MIN) / (BATTERY_VOLTAGE_MAX - BATTERY_VOLTAGE_MIN);
}

static bool battery_adc_init(void)
{
    if (s_adc_handle)
    {
        return true; // already inited
    }

    // Create oneshot ADC unit
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_cfg, &s_adc_handle));

    // Configure channel
    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = BATTERY_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc_handle, BATTERY_ADC_CHANNEL, &chan_cfg));

    // Attempt calibration
    adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id = ADC_UNIT_1,
        .chan = BATTERY_ADC_CHANNEL,
        .atten = BATTERY_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    if (adc_cali_create_scheme_curve_fitting(&cali_cfg, &s_cali_handle) == ESP_OK)
    {
        s_cali_enabled = true;
        ESP_LOGI(TAG, "ADC calibration (curve fitting) enabled");
    }
    else
    {
        s_cali_enabled = false;
        ESP_LOGW(TAG, "ADC calibration not available, using approximate conversion");
    }

    ESP_LOGI(TAG, "Battery ADC init (oneshot)");
    return true;
}

static float read_voltage_multisample(void)
{
    if (!s_adc_handle)
        return -1.0f;
    int raw_sum = 0;
    int raw = 0;
    for (int i = 0; i < BATTERY_MULTISAMPLE_COUNT; ++i)
    {
        if (adc_oneshot_read(s_adc_handle, BATTERY_ADC_CHANNEL, &raw) == ESP_OK)
        {
            raw_sum += raw;
        }
    }
    int raw_avg = raw_sum / BATTERY_MULTISAMPLE_COUNT;

    int mv = 0;
    float volts = 0.0f;
    if (s_cali_enabled && adc_cali_raw_to_voltage(s_cali_handle, raw_avg, &mv) == ESP_OK)
    {
        volts = (mv / 1000.0f) * BATTERY_DIVIDER_FACTOR;
    }
    else
    {
        // fallback approximate conversion (assume ~1100mV reference at 12-bit full scale)
        const float ref_mv = 1100.0f;
        float mv_approx = (raw_avg / 4095.0f) * ref_mv;
        volts = (mv_approx / 1000.0f) * BATTERY_DIVIDER_FACTOR;
    }
    return volts;
}

static void battery_task(void *arg)
{
    (void)arg;
    if (!battery_adc_init())
    {
        ESP_LOGE(TAG, "ADC init failed");
        vTaskDelete(NULL);
        return;
    }
    const TickType_t period = pdMS_TO_TICKS(BATTERY_SAMPLE_PERIOD_MS);
    TickType_t last = xTaskGetTickCount();
    float v = read_voltage_multisample();
    s_voltage_v = v;
    s_percentage = map_voltage_to_percent(v);
    s_valid = true;
    while (1)
    {
        vTaskDelayUntil(&last, period);
        // One multisampled reading (32 raw samples averaged inside helper)
        v = read_voltage_multisample();
        s_voltage_v += BATTERY_EXP_ALPHA * (v - s_voltage_v);
        s_percentage = map_voltage_to_percent(s_voltage_v);
        ESP_LOGD(TAG, "Vbatt=%.3fV pct=%.1f", s_voltage_v, s_percentage * 100.0f);

        if (s_percentage <= 0.10f && !s_inact_forced)
        {
            conf_inact_timeout_s = 3; // permanently shorten until power cycle
            s_inact_forced = true;
            ESP_LOGW(TAG, "Battery low (%.1f%%) -> forcing inactivity timeout to 3s", s_percentage * 100.0f);
        }
        if (s_percentage <= 0.001f)
        {
            ESP_LOGE(TAG, "Battery depleted (%.3fV) -> entering deep sleep to prevent over-discharge", s_voltage_v);
            conf_inact_timeout_s = 0;
            baro_power_down();
            vTaskDelay(pdMS_TO_TICKS(3000));
            esp_deep_sleep_start(); // no return
        }
    }
}

void battery_init(void)
{
    if (!s_task)
    {
        xTaskCreate(battery_task, BATTERY_TASK_NAME, BATTERY_TASK_STACK, NULL, BATTERY_TASK_PRIO, &s_task);
        ESP_LOGI(TAG, "Battery monitor started");
    }
}

float battery_get(void)
{
    if (!s_valid)
        return -0.01f;
    return s_percentage;
}

float battery_get_voltage(void)
{
    if (!s_valid)
        return -1.0f;
    return s_voltage_v;
}
