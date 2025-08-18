// vario.c - Variometer implementation
// Computes vertical speed from barometric pressure and drives a buzzer.

#include "vario.h"

#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "baro.h"
#include "bt.h"
#include "config.h"

// Buzzer type selection:
// Define VARIO_BUZZER_ACTIVE=1 (e.g. add -DVARIO_BUZZER_ACTIVE=1 to CFLAGS) if using a self-oscillating (active) buzzer
// that only needs ON/OFF. Leave undefined or =0 for a passive transducer (default) driven by PWM (LEDC) with variable frequency.
#ifndef VARIO_BUZZER_ACTIVE
#define VARIO_BUZZER_ACTIVE 0
#endif

// Configuration
#define VARIO_TASK_NAME "vario"
#define VARIO_TASK_STACK 4096
#define VARIO_TASK_PRIO 4
#define VARIO_SAMPLE_PERIOD_MS 50  // 20 Hz nominal loop
#define VARIO_MIN_CLIMB_TONE 0.20f // m/s start climb beeps
#define VARIO_MAX_CLIMB_TONE 5.00f
#define VARIO_SINK_ALARM 2.00f // continuous sink tone threshold
#define VARIO_SINK_EXCESS_MAX 5.0f

// Kalman filter tuning (altitude from baro only):
// State x = [ altitude (m); vertical_speed (m/s) ]
// Constant-velocity model with white acceleration noise σ_a.
// Choose σ_a to allow responsiveness to real climb/sink while rejecting noise.
#define VARIO_KF_ACCEL_STD 1.2f     // m/s^2 (process accel noise std dev)
#define VARIO_KF_MEAS_STD_BASE 0.8f // m (base measurement noise std dev)

// Optional adaptive measurement scaling based on innovation magnitude.
#define VARIO_KF_ADAPT_FACTOR_MAX 4.0f // cap multiplier
#define VARIO_KF_INNOVATION_NORM 3.0f  // innovation/σ threshold for scaling

// Additional light smoothing of velocity for audio feel (first-order low-pass tau in ms).
#define VARIO_VS_AUDIO_TAU_MS 180.0f

// Audio parameters
#define VARIO_BEEP_PERIOD_MIN_MS 200.0f
#define VARIO_BEEP_PERIOD_MAX_MS 1000.0f
#define VARIO_BEEP_ON_MIN 0.10f
#define VARIO_BEEP_ON_MAX 0.50f

#define VARIO_BUZZER_GPIO_A 10
#define VARIO_BUZZER_GPIO_B 20
#define VARIO_LEDC_TIMER LEDC_TIMER_0
#define VARIO_LEDC_MODE LEDC_LOW_SPEED_MODE
#define VARIO_LEDC_CHANNEL_A LEDC_CHANNEL_0
#define VARIO_LEDC_CHANNEL_B LEDC_CHANNEL_1
#define VARIO_LEDC_DUTY_RES LEDC_TIMER_8_BIT
#define VARIO_CLIMB_FREQ_MIN 1000.0f
#define VARIO_CLIMB_FREQ_MAX 3000.0f
#define VARIO_CLIMB_BUZZER_DUTY 0.5f
#define VARIO_SINK_FREQ_MAX 400.0f
#define VARIO_SINK_FREQ_MIN 200.0f
#define VARIO_SINK_BUZZER_DUTY 0.5f
#define VARIO_BUZZER_RESONANT_FREQ 2100.0f

static const char *TAG = "VARIO";

static TaskHandle_t s_task_handle = NULL;
static float s_vspeed = 0.0f;     // public vertical speed (filtered for audio)
static float s_vspeed_raw = 0.0f; // raw Kalman vertical speed
static float s_altitude = 0.0f;   // current altitude estimate
static double s_last_pressure = 0.0;
static double s_last_temperature = 0.0;
static bool s_sample_valid = false;
static bool s_kf_initialized = false;
static double s_p0 = 101325.0; // reference pressure Pa captured at init

// Kalman covariance matrix P (2x2):
static float P00 = 4.0f, P01 = 0.0f, P10 = 0.0f, P11 = 4.0f; // start with generous uncertainty

static inline float pressure_to_altitude(double pressure_pa)
{
    if (pressure_pa <= 0.0)
        return s_altitude; // guard
    float ratio = (float)(s_p0 / pressure_pa);
    float expv = powf(ratio, 0.190284f) - 1.0f; // (1/5.2558797)
    return 44330.77f * expv;                    // meters
}

#if !VARIO_BUZZER_ACTIVE
static void buzzer_set(float freq_hz, float duty)
{
    // Differential drive: two LEDC channels, second inverted, sharing same timer.
    // This doubles peak-to-peak voltage across the transducer for same supply.
    if (freq_hz <= 0.0f || duty <= 0.0f)
    {
        ledc_stop(VARIO_LEDC_MODE, VARIO_LEDC_CHANNEL_A, 0);
        ledc_stop(VARIO_LEDC_MODE, VARIO_LEDC_CHANNEL_B, 1);
        return;
    }
    if (freq_hz > 5000.0f)
        freq_hz = 5000.0f;
    ledc_set_freq(VARIO_LEDC_MODE, VARIO_LEDC_TIMER, (uint32_t)freq_hz);
    const float max_duty = (1u << VARIO_LEDC_DUTY_RES) - 1u;
    uint32_t duty_val = (uint32_t)(duty * max_duty);
    ledc_set_duty(VARIO_LEDC_MODE, VARIO_LEDC_CHANNEL_A, duty_val);
    ledc_set_duty(VARIO_LEDC_MODE, VARIO_LEDC_CHANNEL_B, duty_val);
    ledc_update_duty(VARIO_LEDC_MODE, VARIO_LEDC_CHANNEL_A);
    ledc_update_duty(VARIO_LEDC_MODE, VARIO_LEDC_CHANNEL_B);
}
#else
static void buzzer_set(float freq_hz, float duty)
{
    // Active buzzer: ignore frequency and duty specifics; treat any non-zero request as ON
    if (freq_hz <= 0.0f || duty <= 0.0f)
    {
        gpio_hold_dis(VARIO_BUZZER_GPIO_A);
        gpio_set_level(VARIO_BUZZER_GPIO_A, 0);
        gpio_hold_en(VARIO_BUZZER_GPIO_A);
    }
    else
    {
        gpio_hold_dis(VARIO_BUZZER_GPIO_A);
        gpio_set_level(VARIO_BUZZER_GPIO_A, 1);
        gpio_hold_en(VARIO_BUZZER_GPIO_A);
    }
}
#endif

// Unified timer-based scheduling
static esp_timer_handle_t s_beep_cycle_timer = NULL; // schedules next beep start
static esp_timer_handle_t s_beep_off_timer = NULL;   // turns buzzer off in current cycle
static bool s_in_climb_mode = false;
static bool s_beep_on = false; // only meaningful for passive (LED PWM active)

// ---- Utility helpers (after macros) ----
static inline float clampf(float v, float lo, float hi) { return (v < lo) ? lo : (v > hi) ? hi
                                                                                          : v; }

static void cancel_beep_timers(void)
{
    if (s_beep_cycle_timer)
        esp_timer_stop(s_beep_cycle_timer);
    if (s_beep_off_timer)
        esp_timer_stop(s_beep_off_timer);
    s_beep_on = false;
    s_in_climb_mode = false;
}

static void beep_off_cb(void *arg)
{
    (void)arg;
    s_beep_on = false;
    buzzer_set(0, 0); // stop tone or turn off active buzzer
}

static void beep_cycle_cb(void *arg)
{
    (void)arg;

    // Map vs -> frac 0..1
    float climb = clampf(s_vspeed, VARIO_MIN_CLIMB_TONE, VARIO_MAX_CLIMB_TONE);
    float frac = (climb - VARIO_MIN_CLIMB_TONE) / (VARIO_MAX_CLIMB_TONE - VARIO_MIN_CLIMB_TONE);
    frac = clampf(frac, 0.0f, 1.0f);

    float period_ms = VARIO_BEEP_PERIOD_MAX_MS - frac * (VARIO_BEEP_PERIOD_MAX_MS - VARIO_BEEP_PERIOD_MIN_MS);
    period_ms = clampf(period_ms, VARIO_BEEP_PERIOD_MIN_MS, VARIO_BEEP_PERIOD_MAX_MS);

    float on_duty = VARIO_BEEP_ON_MIN + frac * (VARIO_BEEP_ON_MAX - VARIO_BEEP_ON_MIN);
    on_duty = clampf(on_duty, VARIO_BEEP_ON_MIN, VARIO_BEEP_ON_MAX);

    uint64_t period_us = (uint64_t)(period_ms * 1000.0f);
    uint64_t on_us = (uint64_t)(period_ms * on_duty * 1000.0f);
    if (on_us < 1000)
        on_us = 1000;
#if VARIO_BUZZER_ACTIVE
    buzzer_set(1, 1);
#else
    float freq = VARIO_CLIMB_FREQ_MIN + frac * (VARIO_CLIMB_FREQ_MAX - VARIO_CLIMB_FREQ_MIN);
    buzzer_set(freq, VARIO_CLIMB_BUZZER_DUTY);
#endif
    s_beep_on = true;
    if (s_beep_off_timer)
    {
        esp_timer_stop(s_beep_off_timer);
        esp_timer_start_once(s_beep_off_timer, on_us);
    }
    if (s_beep_cycle_timer)
    {
        esp_timer_start_once(s_beep_cycle_timer, period_us);
    }
}

static void update_audio(void)
{
    // Sink alarm continuous tone
    if (conf_enable_audio && !bt_is_connected() && s_vspeed <= -VARIO_SINK_ALARM)
    {
        cancel_beep_timers();
#if VARIO_BUZZER_ACTIVE
        buzzer_set(1, 1);
#else
        float sink_excess = fminf(VARIO_SINK_EXCESS_MAX, (-s_vspeed) - (VARIO_SINK_ALARM));
        float frac = sink_excess / VARIO_SINK_EXCESS_MAX; // 0..1
        float freq = VARIO_SINK_FREQ_MAX - frac * (VARIO_SINK_FREQ_MAX - VARIO_SINK_FREQ_MIN);
        buzzer_set(freq, VARIO_SINK_BUZZER_DUTY);
#endif
        return;
    }
    // Neutral / disabled
    if (!conf_enable_audio || bt_is_connected() || s_vspeed < VARIO_MIN_CLIMB_TONE)
    {
        cancel_beep_timers();
        buzzer_set(0, 0);
        return;
    }
    // Enter climb mode if needed
    if (!s_in_climb_mode)
    {
        s_in_climb_mode = true;
        beep_cycle_cb(NULL);
        return;
    }
#if !VARIO_BUZZER_ACTIVE
    // Passive: update frequency during ON phase
    if (s_beep_on)
    {
        float climb = clampf(s_vspeed, VARIO_MIN_CLIMB_TONE, VARIO_MAX_CLIMB_TONE);
        float frac = (climb - VARIO_MIN_CLIMB_TONE) / (VARIO_MAX_CLIMB_TONE - VARIO_MIN_CLIMB_TONE);
        float freq = VARIO_CLIMB_FREQ_MIN + frac * (VARIO_CLIMB_FREQ_MAX - VARIO_CLIMB_FREQ_MIN);
        buzzer_set(freq, VARIO_CLIMB_BUZZER_DUTY);
    }
#endif
}

static void vario_task(void *arg)
{
    (void)arg;
    int64_t prev_us = esp_timer_get_time();
    double pressure = 0.0, temperature = 0.0;

    const TickType_t period = pdMS_TO_TICKS(VARIO_SAMPLE_PERIOD_MS);
    TickType_t last_wake_time = xTaskGetTickCount();
    while (1)
    {
        vTaskDelayUntil(&last_wake_time, period);
        baro_read(&pressure, &temperature);
        if (!s_kf_initialized)
        {
            if (pressure > 1000.0)
            {
                s_p0 = pressure; // reference
                s_altitude = pressure_to_altitude(pressure);
                s_vspeed_raw = 0.0f;
                s_vspeed = 0.0f;
                // Reset covariance
                P00 = 10.0f;
                P01 = 0.0f;
                P10 = 0.0f;
                P11 = 10.0f;
                s_kf_initialized = true;
                prev_us = esp_timer_get_time();
            }
        }
        else
        {
            int64_t now_us = esp_timer_get_time();
            float dt = (float)(now_us - prev_us) / 1e6f; // seconds
            if (dt < 0.005f)
                dt = VARIO_SAMPLE_PERIOD_MS / 1000.0f; // fallback
            if (dt > 0.5f)
                dt = VARIO_SAMPLE_PERIOD_MS / 1000.0f;

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
            float Q01 = 0.5f * dt3 * sa2;
            float Q11 = dt2 * sa2;

            // Covariance prediction: P = F P F^T + Q; F=[[1 dt],[0 1]]
            float P00_new = P00 + dt * (P10 + P01) + dt2 * P11 + Q00;
            float P01_new = P01 + dt * P11 + Q01;
            float P10_new = P10 + dt * P11 + Q01; // symmetric plus Q01
            float P11_new = P11 + Q11;
            P00 = P00_new;
            P01 = P01_new;
            P10 = P10_new;
            P11 = P11_new;

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
            if (innov_norm > VARIO_KF_INNOVATION_NORM)
            {
                float scale = innov_norm / VARIO_KF_INNOVATION_NORM;
                if (scale > VARIO_KF_ADAPT_FACTOR_MAX)
                    scale = VARIO_KF_ADAPT_FACTOR_MAX;
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
            P00 = P00_post;
            P01 = P01_post;
            P10 = P10_post;
            P11 = P11_post;

            // 3. Light smoothing for vertical speed used for audio (separate from estimation accuracy)
            float dt_ms = dt * 1000.0f;
            float alpha_vs = dt_ms / (VARIO_VS_AUDIO_TAU_MS + dt_ms);
            s_vspeed += alpha_vs * (s_vspeed_raw - s_vspeed);

            // TEST: Simulate vertical speed patterns to exercise audio without real sensor motion.
            // Replaces real vspeed audio drive with synthetic profile cycling through sink, neutral, and climb.
            {
                static float sim_t = 0.0f;
                sim_t += dt;               // advance simulation time (seconds)
                const float cycle = 42.0f; // total cycle length (s)
                if (sim_t > cycle)
                    sim_t -= cycle;

                float test_vs = 0.0f; // synthetic vertical speed (m/s)

                if (sim_t < 4.0f)
                {
                    // Near zero with gentle oscillation: ~ +/-0.5 m/s
                    float r = sim_t / 4.0f;
                    test_vs = 1.0f * sinf(r * 2.0f * (float)M_PI);
                }
                else if (sim_t < 20.0f)
                {
                    // Climb ramp: 0 -> +3 m/s
                    float r = (sim_t - 4.0f) / 16.0f;
                    test_vs = r * 3.0f;
                }
                else if (sim_t < 24.0f)
                {
                    // Hold strong climb: +3 m/s
                    test_vs = 3.0f;
                }
                else if (sim_t < 28.0f)
                {
                    // Decay climb: +3 -> +1 m/s
                    float r = (sim_t - 24.0f) / 4.0f;
                    test_vs = 3.0f - r * 2.0f;
                }
                else if (sim_t < 32.0f)
                {
                    // Near zero with gentle oscillation: ~ +/-0.5 m/s
                    float r = (sim_t - 28.0f) / 4.0f;
                    test_vs = 0.8f * sinf(r * 2.0f * (float)M_PI);
                }
                else if (sim_t < 36.0f)
                {
                    // Sink ramp: 0 -> -7 m/s (tests sink alarm trigger)
                    float r = (sim_t - 32.0f) / 4.0f;
                    test_vs = -r * 7.0f;
                }
                else if (sim_t < 40.0f)
                {
                    // -7 m/s -> 0
                    float r = (sim_t - 36.0f) / 4.0f;
                    test_vs = -7.0f + r * 7.0f;
                }
                else
                {
                    // Recover with slight negative oscillation around -0.5..+0.5 m/s
                    float r = (sim_t - 40.0f) / 4.0f;
                    test_vs = 0.6f * sinf(r * 2.0f * (float)M_PI);
                }

                // Override reported vspeed so vario_get() reflects the test pattern.
                s_vspeed = test_vs;
            }
            update_audio();
            prev_us = now_us;
        }
    }
}

void vario_init(void)
{
    gpio_hold_dis(VARIO_BUZZER_GPIO_A);
    gpio_hold_dis(VARIO_BUZZER_GPIO_B);
    // Hardware init depends on buzzer type
#if !VARIO_BUZZER_ACTIVE
    ledc_timer_config_t timer_cfg = {
        .speed_mode = VARIO_LEDC_MODE,
        .duty_resolution = VARIO_LEDC_DUTY_RES,
        .timer_num = VARIO_LEDC_TIMER,
        .freq_hz = 1000,
        .clk_cfg = LEDC_USE_RC_FAST_CLK,
    };
    ledc_timer_config(&timer_cfg);

    // Primary (non-inverted) channel
    ledc_channel_config_t ch_cfg_a = {
        .gpio_num = VARIO_BUZZER_GPIO_A,
        .speed_mode = VARIO_LEDC_MODE,
        .channel = VARIO_LEDC_CHANNEL_A,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = VARIO_LEDC_TIMER,
        .duty = 0,
        .hpoint = 0,
        .flags.output_invert = 0,
        .sleep_mode = LEDC_SLEEP_MODE_KEEP_ALIVE,
    };
    ledc_channel_config(&ch_cfg_a);
    // Secondary inverted channel for push-pull
    ledc_channel_config_t ch_cfg_b = {
        .gpio_num = VARIO_BUZZER_GPIO_B,
        .speed_mode = VARIO_LEDC_MODE,
        .channel = VARIO_LEDC_CHANNEL_B,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = VARIO_LEDC_TIMER,
        .duty = 0,
        .hpoint = 0,
        .flags.output_invert = 1, // invert output!
        .sleep_mode = LEDC_SLEEP_MODE_KEEP_ALIVE,
    };
    ledc_channel_config(&ch_cfg_b);
#else
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << VARIO_BUZZER_GPIO_A,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = 0,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);
    gpio_set_level(VARIO_BUZZER_GPIO_A, 0);
    gpio_hold_en(VARIO_BUZZER_GPIO_A);
#endif

    // Create (or reuse) timers for unified scheduling
    if (!s_beep_cycle_timer)
    {
        const esp_timer_create_args_t cycle_args = {
            .callback = &beep_cycle_cb,
            .arg = NULL,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "beep_cycle",
        };
        esp_timer_create(&cycle_args, &s_beep_cycle_timer);
    }
    if (!s_beep_off_timer)
    {
        const esp_timer_create_args_t off_args = {
            .callback = &beep_off_cb,
            .arg = NULL,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "beep_off",
        };
        esp_timer_create(&off_args, &s_beep_off_timer);
    }

    gpio_set_drive_capability(VARIO_BUZZER_GPIO_A, GPIO_DRIVE_CAP_3);
    gpio_set_drive_capability(VARIO_BUZZER_GPIO_B, GPIO_DRIVE_CAP_3);

    xTaskCreate(vario_task, VARIO_TASK_NAME, VARIO_TASK_STACK, NULL, VARIO_TASK_PRIO, &s_task_handle);
#if VARIO_BUZZER_ACTIVE
    ESP_LOGI(TAG, "Variometer initialized (ACTIVE buzzer GPIO %d)", VARIO_BUZZER_GPIO_A);
#else
    ESP_LOGI(TAG, "Variometer initialized (passive buzzer differential PWM GPIO %d/%d)", VARIO_BUZZER_GPIO_A, VARIO_BUZZER_GPIO_B);
#endif
}

bool vario_get(vario_data_t *out)
{
    if (!out || !s_sample_valid)
    {
        return false;
    }
    out->pressure_pa = s_last_pressure;
    out->temperature_c = s_last_temperature;
    out->altitude_m = s_altitude;
    out->vspeed_mps = s_vspeed;
    return true;
}
