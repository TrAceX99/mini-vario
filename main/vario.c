// vario.c - Variometer implementation
// Computes vertical speed from barometric pressure and drives a buzzer.

#include "vario.h"

#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/ledc.h"
#include "baro.h"
#include "bt.h"
#include "config.h"

// Configuration
#define VARIO_TASK_NAME            "vario"
#define VARIO_TASK_STACK           2048
#define VARIO_TASK_PRIO            4
#define VARIO_SAMPLE_PERIOD_MS     50            // 20 Hz nominal loop
#define VARIO_MIN_CLIMB_TONE       0.30f         // m/s start climb beeps
#define VARIO_MAX_CLIMB_TONE       5.00f
#define VARIO_SINK_ALARM           -2.00f        // continuous sink tone threshold
#define VARIO_BUZZER_GPIO          10            // GPIO for buzzer

// Kalman filter tuning (altitude from baro only):
// State x = [ altitude (m); vertical_speed (m/s) ]
// Constant-velocity model with white acceleration noise σ_a.
// Choose σ_a to allow responsiveness to real climb/sink while rejecting noise.
#define VARIO_KF_ACCEL_STD         1.2f          // m/s^2 (process accel noise std dev)
#define VARIO_KF_MEAS_STD_BASE     0.8f          // m (base measurement noise std dev)

// Optional adaptive measurement scaling based on innovation magnitude.
#define VARIO_KF_ADAPT_FACTOR_MAX  4.0f          // cap multiplier
#define VARIO_KF_INNOVATION_NORM   3.0f          // innovation/σ threshold for scaling

// Additional light smoothing of velocity for audio feel (first-order low-pass tau in ms).
#define VARIO_VS_AUDIO_TAU_MS      180.0f

// LEDC
#define VARIO_LEDC_TIMER         LEDC_TIMER_0
#define VARIO_LEDC_MODE          LEDC_LOW_SPEED_MODE
#define VARIO_LEDC_CHANNEL       LEDC_CHANNEL_0
#define VARIO_LEDC_DUTY_RES      LEDC_TIMER_10_BIT

static const char *TAG = "VARIO";

static TaskHandle_t s_task_handle = NULL;
static float s_vspeed = 0.0f;          // public vertical speed (filtered for audio)
static float s_vspeed_raw = 0.0f;      // raw Kalman vertical speed
static float s_altitude = 0.0f;        // current altitude estimate
static double s_last_pressure = 0.0;
static double s_last_temperature = 0.0;
static bool   s_sample_valid = false;
static bool  s_kf_initialized = false;
static double s_p0 = 101325.0;         // reference pressure Pa captured at init

// Kalman covariance matrix P (2x2):
static float P00 = 4.0f, P01 = 0.0f, P10 = 0.0f, P11 = 4.0f; // start with generous uncertainty

static inline float pressure_to_altitude(double pressure_pa)
{
    if (pressure_pa <= 0.0) return s_altitude; // guard
    float ratio = (float)(s_p0 / pressure_pa);
    float expv = powf(ratio, 0.190284f) - 1.0f; // (1/5.2558797)
    return 44330.77f * expv; // meters
}

static void buzzer_set(float freq_hz, float duty)
{
    if (freq_hz <= 0.0f || duty <= 0.0f) {
        ledc_stop(VARIO_LEDC_MODE, VARIO_LEDC_CHANNEL, 0);
        return;
    }
    if (freq_hz > 4000.0f) freq_hz = 4000.0f;
    ledc_set_freq(VARIO_LEDC_MODE, VARIO_LEDC_TIMER, (uint32_t)freq_hz);
    const float max_duty = (1u << VARIO_LEDC_DUTY_RES) - 1u;
    uint32_t duty_val = (uint32_t)(duty * max_duty);
    ledc_set_duty(VARIO_LEDC_MODE, VARIO_LEDC_CHANNEL, duty_val);
    ledc_update_duty(VARIO_LEDC_MODE, VARIO_LEDC_CHANNEL);
}

static void update_audio(float vs)
{
    if (!conf_enable_audio || !bt_is_connected())
    {
        buzzer_set(0, 0);
        return;
    }

    if (vs >= VARIO_MIN_CLIMB_TONE) {
        float climb = vs;
        if (climb > VARIO_MAX_CLIMB_TONE) climb = VARIO_MAX_CLIMB_TONE;
        float freq = 800.0f + (climb - VARIO_MIN_CLIMB_TONE) / (VARIO_MAX_CLIMB_TONE - VARIO_MIN_CLIMB_TONE) * 1200.0f; // 800-2000 Hz
        float cycle_ms = 800.0f - (climb - VARIO_MIN_CLIMB_TONE) / (VARIO_MAX_CLIMB_TONE - VARIO_MIN_CLIMB_TONE) * 600.0f; // 800->200
        float on_ms = cycle_ms * 0.35f + climb * 20.0f;
        uint64_t t_ms = esp_timer_get_time() / 1000ULL;
        if ((t_ms % (uint64_t)cycle_ms) < (uint64_t)on_ms) {
            buzzer_set(freq, 0.5f);
        } else {
            buzzer_set(0, 0);
        }
    } else if (vs <= VARIO_SINK_ALARM) {
        float sink_excess = fminf(5.0f, (-vs) - (-VARIO_SINK_ALARM));
        float freq = 400.0f - sink_excess * 30.0f; // 400 downwards
        if (freq < 200.0f) freq = 200.0f;
        buzzer_set(freq, 0.3f);
    } else {
        buzzer_set(0, 0);
    }
}

static void vario_task(void *arg)
{
    (void)arg;
    int64_t prev_us = esp_timer_get_time();
    double pressure = 0.0, temperature = 0.0;

    while (1) {
        baro_read(&pressure, &temperature);
        if (!s_kf_initialized) {
            if (pressure > 1000.0) {
                s_p0 = pressure; // reference
                s_altitude = pressure_to_altitude(pressure);
                s_vspeed_raw = 0.0f;
                s_vspeed = 0.0f;
                // Reset covariance
                P00 = 10.0f; P01 = 0.0f; P10 = 0.0f; P11 = 10.0f;
                s_kf_initialized = true;
                prev_us = esp_timer_get_time();
            }
        } else {
            int64_t now_us = esp_timer_get_time();
            float dt = (float)(now_us - prev_us) / 1e6f; // seconds
            if (dt < 0.005f) dt = VARIO_SAMPLE_PERIOD_MS / 1000.0f; // fallback
            if (dt > 0.5f)   dt = VARIO_SAMPLE_PERIOD_MS / 1000.0f;

            // 1. Prediction step
            // State prediction
            s_altitude += s_vspeed_raw * dt; // h = h + v*dt
            // v unchanged

            // Process noise (discrete) from accel σ_a
            float sa2 = VARIO_KF_ACCEL_STD * VARIO_KF_ACCEL_STD;
            float dt2 = dt * dt;
            float dt3 = dt2 * dt;
            float dt4 = dt2 * dt2;
            float Q00 = 0.25f * dt4 * sa2;
            float Q01 = 0.5f  * dt3 * sa2;
            float Q11 = dt2 * sa2;

            // Covariance prediction: P = F P F^T + Q; F=[[1 dt],[0 1]]
            float P00_new = P00 + dt*(P10 + P01) + dt2*P11 + Q00;
            float P01_new = P01 + dt*P11 + Q01;
            float P10_new = P10 + dt*P11 + Q01; // symmetric plus Q01
            float P11_new = P11 + Q11;
            P00 = P00_new; P01 = P01_new; P10 = P10_new; P11 = P11_new;

            // 2. Measurement update (z = altitude from pressure)
            float z = pressure_to_altitude(pressure);
            s_last_pressure = pressure;
            s_last_temperature = temperature;
            s_sample_valid = true;
            // Innovation
            float y = z - s_altitude;

            // Adaptive measurement variance scaling based on innovation
            float R = VARIO_KF_MEAS_STD_BASE * VARIO_KF_MEAS_STD_BASE;
            float innov_norm = fabsf(y) / (sqrtf(R) + 1e-6f);
            if (innov_norm > VARIO_KF_INNOVATION_NORM) {
                float scale = innov_norm / VARIO_KF_INNOVATION_NORM;
                if (scale > VARIO_KF_ADAPT_FACTOR_MAX) scale = VARIO_KF_ADAPT_FACTOR_MAX;
                R *= scale; // increase R => trust prediction more for large spikes
            }

            float S = P00 + R; // scalar innovation covariance
            float invS = 1.0f / (S + 1e-9f);
            float K0 = P00 * invS;
            float K1 = P10 * invS;

            // State update
            s_altitude += K0 * y;
            s_vspeed_raw += K1 * y;

            // Covariance update: (I-KH)P, H=[1 0]
            float P00_post = (1.0f - K0) * P00;
            float P01_post = (1.0f - K0) * P01;
            float P10_post = P10 - K1 * P00; // or (1-K0)*P10
            float P11_post = P11 - K1 * P01; // reduces correlation appropriately
            P00 = P00_post; P01 = P01_post; P10 = P10_post; P11 = P11_post;

            // 3. Light smoothing for audio output (separate from estimation accuracy)
            float dt_ms = dt * 1000.0f;
            float alpha_vs = dt_ms / (VARIO_VS_AUDIO_TAU_MS + dt_ms);
            s_vspeed += alpha_vs * (s_vspeed_raw - s_vspeed);

            update_audio(s_vspeed);
            prev_us = now_us;
        }

        vTaskDelay(pdMS_TO_TICKS(VARIO_SAMPLE_PERIOD_MS));
    }
}

void vario_init(void)
{
    if (s_task_handle) return;

    ledc_timer_config_t timer_cfg = {
        .speed_mode = VARIO_LEDC_MODE,
        .duty_resolution = VARIO_LEDC_DUTY_RES,
        .timer_num = VARIO_LEDC_TIMER,
        .freq_hz = 1000,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&timer_cfg);

    ledc_channel_config_t ch_cfg = {
        .gpio_num = VARIO_BUZZER_GPIO,
        .speed_mode = VARIO_LEDC_MODE,
        .channel = VARIO_LEDC_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = VARIO_LEDC_TIMER,
        .duty = 0,
        .hpoint = 0,
        .flags.output_invert = 0,
    };
    ledc_channel_config(&ch_cfg);

    xTaskCreate(vario_task, VARIO_TASK_NAME, VARIO_TASK_STACK, NULL, VARIO_TASK_PRIO, &s_task_handle);
    ESP_LOGI(TAG, "Variometer initialized (buzzer GPIO %d)", VARIO_BUZZER_GPIO);
}

bool vario_get(vario_data_t *out)
{
    if (!out || !s_sample_valid) return false;
    out->pressure_pa = s_last_pressure;
    out->temperature_c = s_last_temperature;
    out->altitude_m = s_altitude;
    out->vspeed_mps = s_vspeed;
    return true;
}
