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
#define VARIO_MIN_SINK_TONE 2.00f // continuous sink tone threshold
#define VARIO_MAX_SINK_TONE 5.0f

// Light pressure smoothing (low intensity) ONLY for reported pressure (vario_get),
// not used internally for Kalman / altitude calculations. Time constant ~100 ms.
#define VARIO_PRESSURE_FILTER_TAU_S 0.1f

// Kalman filter tuning (altitude from baro only):
// State x = [ altitude (m); vertical_speed (m/s) ]
// Constant-velocity model with white acceleration noise σ_a.
// Choose σ_a to allow responsiveness to real climb/sink while rejecting noise.
#define VARIO_KF_ACCEL_STD 1.5f      // m/s^2 (process accel noise std dev)
#define VARIO_KF_MEAS_STD_BASE 0.75f // m (base measurement noise std dev)

// Optional adaptive measurement scaling based on innovation magnitude.
#define VARIO_KF_ADAPT_FACTOR_MAX 4.0f // cap multiplier
#define VARIO_KF_INNOVATION_NORM 3.0f  // innovation/σ threshold for scaling

// Additional light smoothing of velocity for audio feel (first-order low-pass tau in ms).
#define VARIO_VS_AUDIO_TAU_MS 180.0f

// Audio parameters
#define VARIO_BEEP_PERIOD_MIN_MS 150.0f
#define VARIO_BEEP_PERIOD_MAX_MS 500.0f
#define VARIO_BEEP_ON_MIN 0.10f
#define VARIO_BEEP_ON_MAX 0.40f

#define VARIO_BUZZER_GPIO_A 10
#define VARIO_BUZZER_GPIO_B 20
#define VARIO_LEDC_TIMER LEDC_TIMER_0
#define VARIO_LEDC_MODE LEDC_LOW_SPEED_MODE
#define VARIO_LEDC_CHANNEL_A LEDC_CHANNEL_0
#define VARIO_LEDC_CHANNEL_B LEDC_CHANNEL_1
#define VARIO_LEDC_DUTY_RES LEDC_TIMER_10_BIT
#define VARIO_CLIMB_FREQ_MIN 1700.0f
#define VARIO_CLIMB_FREQ_MAX 2500.0f
#define VARIO_SINK_FREQ_MAX 400.0f
#define VARIO_SINK_FREQ_MIN 200.0f
#define VARIO_BUZZER_RESONANT_FREQ 2000.0f
#define VARIO_BUZZER_PWM_DUTY 0.5f

// Inactivity (anti-forgetfulness) reminder: single short beep if neutral & silent for this long
#define VARIO_INACTIVITY_TIMEOUT_S 100 // seconds of no climb/sink activity

static const char *TAG = "VARIO";

static TaskHandle_t s_task_handle = NULL;
static float s_vspeed = 0.0f;     // public vertical speed (filtered for audio)
static float s_vspeed_raw = 0.0f; // raw Kalman vertical speed
static float s_altitude = 0.0f;   // current altitude estimate
static float s_last_pressure = 0.0f;
static float s_last_temperature = 0.0f;
static bool s_sample_valid = false;
static bool s_kf_initialized = false;
static float s_p0 = 101325.0f;         // reference pressure Pa captured at init
static float s_pressure_filt = 0.0f;   // exponentially smoothed pressure (Pa)
static int64_t s_last_activity_us = 0; // last time we had meaningful vertical activity or emitted reminder

// Kalman covariance matrix P (2x2):
static float P00 = 4.0f, P01 = 0.0f, P10 = 0.0f, P11 = 4.0f; // start with generous uncertainty

// Audio state machine -------------------------------------------------------
typedef enum
{
    VARIO_AUDIO_DISABLED = 0, // Audio muted due to config or BT connection
    VARIO_AUDIO_NEUTRAL,      // Silence (between sink alarm and climb beep range)
    VARIO_AUDIO_SINK,         // Continuous sink tone
    VARIO_AUDIO_CLIMB,        // Intermittent climb beeps with ON/OFF pattern
} vario_audio_state_t;

static esp_timer_handle_t s_beep_timer = NULL; // one-shot; rearmed each transition / phase
static vario_audio_state_t s_audio_state = VARIO_AUDIO_DISABLED;
static bool s_beeping = false;

// ---- Utility helpers (after macros) ----
static inline float clampf(float v, float lo, float hi) { return (v < lo) ? lo : (v > hi) ? hi
                                                                                          : v; }

static inline float pressure_to_altitude(float pressure_pa)
{
    if (pressure_pa <= 0.0)
        return 0.0;

    // ISA standard atmosphere constants (troposphere)
    const float T0 = 288.15;  // K
    const float L = 0.0065;   // K/m (temperature lapse rate)
    const float g0 = 9.80665; // m/s^2
    const float R = 287.053;  // J/(kg·K)

    const float exponent = (R * L) / g0; // ≈ 0.190263
    float ratio = s_p0 / pressure_pa;
    float term = powf(ratio, exponent) - 1.0f;
    return (T0 / L) * term; // meters
}

#if !VARIO_BUZZER_ACTIVE
static inline void buzzer_set(float freq_hz)
{
    ledc_set_freq(VARIO_LEDC_MODE, VARIO_LEDC_TIMER, (uint32_t)freq_hz);
}
static inline void buzzer_on(void)
{
    ledc_update_duty(VARIO_LEDC_MODE, VARIO_LEDC_CHANNEL_A);
    ledc_update_duty(VARIO_LEDC_MODE, VARIO_LEDC_CHANNEL_B);
}
static inline void buzzer_off(void)
{
    ledc_stop(VARIO_LEDC_MODE, VARIO_LEDC_CHANNEL_A, 0);
    ledc_stop(VARIO_LEDC_MODE, VARIO_LEDC_CHANNEL_B, 1);
}
#else
static inline void buzzer_set(float freq_hz)
{
    (void)freq_hz;
    return;
}
static inline void buzzer_on(void)
{
    gpio_hold_dis(VARIO_BUZZER_GPIO_A);
    gpio_set_level(VARIO_BUZZER_GPIO_A, 1);
    gpio_hold_en(VARIO_BUZZER_GPIO_A);
}
static inline void buzzer_off(void)
{
    gpio_hold_dis(VARIO_BUZZER_GPIO_A);
    gpio_set_level(VARIO_BUZZER_GPIO_A, 0);
    gpio_hold_en(VARIO_BUZZER_GPIO_A);
}
#endif

static inline float compute_climb_beep_duration_ms(bool is_on_period)
{
    float frac = (s_vspeed - VARIO_MIN_CLIMB_TONE) / (VARIO_MAX_CLIMB_TONE - VARIO_MIN_CLIMB_TONE);
    frac = clampf(frac, 0.0f, 1.0f);
    float period_ms = VARIO_BEEP_PERIOD_MIN_MS + frac * (VARIO_BEEP_PERIOD_MAX_MS - VARIO_BEEP_PERIOD_MIN_MS);
    float on_duty = VARIO_BEEP_ON_MIN + frac * (VARIO_BEEP_ON_MAX - VARIO_BEEP_ON_MIN);
    if (is_on_period)
    {
        return period_ms * on_duty;
    }
    else
    {
        return period_ms * (1.0f - on_duty);
    }
}

static void beep_timer_cb(void *arg)
{
    (void)arg;

    static bool beep_on = false;

    if (esp_timer_is_active(s_beep_timer))
    {
        return;
    }

    if (!s_beeping)
    {
        if (beep_on)
        {
            buzzer_off();
            beep_on = false;
        }
        return;
    }

    if (!beep_on)
    {
        // OFF -> ON
        buzzer_on();
        beep_on = true;
        esp_timer_start_once(s_beep_timer, compute_climb_beep_duration_ms(true) * 1000ULL);
    }
    else
    {
        // ON -> OFF
        buzzer_off();
        beep_on = false;
        esp_timer_start_once(s_beep_timer, compute_climb_beep_duration_ms(false) * 1000ULL);
    }
}

static inline void beep_start(void)
{
    s_beeping = true;
    beep_timer_cb(NULL);
}

static inline void beep_stop(void)
{
    s_beeping = false;
}

static void update_beep_frequency()
{
    float climb_frac = (s_vspeed - VARIO_MIN_CLIMB_TONE) / (VARIO_MAX_CLIMB_TONE - VARIO_MIN_CLIMB_TONE);
    climb_frac = clampf(climb_frac, -0.5f, 1.0f);
    float climb_freq = VARIO_CLIMB_FREQ_MIN + climb_frac * (VARIO_CLIMB_FREQ_MAX - VARIO_CLIMB_FREQ_MIN);
    buzzer_set(climb_freq);
}

static void audio_set_state(vario_audio_state_t st)
{
    vario_audio_state_t prev_state = s_audio_state;
    s_audio_state = st;

    if (prev_state != st && prev_state == VARIO_AUDIO_CLIMB)
    {
        beep_stop();
    }
    if (s_beeping)
    {
        update_beep_frequency();
    }

    switch (st)
    {
    case VARIO_AUDIO_DISABLED:
        if (prev_state != VARIO_AUDIO_DISABLED)
        {
            buzzer_off();
        }
        break;
    case VARIO_AUDIO_NEUTRAL:
        if (prev_state != VARIO_AUDIO_NEUTRAL)
        {
            s_last_activity_us = esp_timer_get_time();
        }
        if (prev_state == VARIO_AUDIO_SINK)
        {
            buzzer_off();
        }

        // Inactivity reminder alarm
        if (esp_timer_get_time() - s_last_activity_us > (VARIO_INACTIVITY_TIMEOUT_S * 1000000ULL))
        {
            // Do a single beep
            buzzer_set(VARIO_BUZZER_RESONANT_FREQ);
            beep_start();
            beep_stop();
            s_last_activity_us = esp_timer_get_time();
        }
        break;
    case VARIO_AUDIO_SINK:
#if !VARIO_BUZZER_ACTIVE
        float sink_frac = (-s_vspeed - VARIO_MIN_SINK_TONE) / (VARIO_MAX_SINK_TONE - VARIO_MIN_SINK_TONE);
        sink_frac = clampf(sink_frac, 0.0f, 1.0f);
        float sink_freq = VARIO_SINK_FREQ_MAX - sink_frac * (VARIO_SINK_FREQ_MAX - VARIO_SINK_FREQ_MIN);
        buzzer_set(sink_freq);
#endif
        buzzer_on();
        break;
    case VARIO_AUDIO_CLIMB:
        if (prev_state != VARIO_AUDIO_CLIMB)
        {
            beep_start();
        }
        break;
    }
}

static void update_audio(void)
{
    if (!conf_enable_audio || bt_is_connected())
    {
        audio_set_state(VARIO_AUDIO_DISABLED);
        return;
    }

    if (s_vspeed <= -VARIO_MIN_SINK_TONE)
    {
        audio_set_state(VARIO_AUDIO_SINK);
    }
    else if (s_vspeed >= VARIO_MIN_CLIMB_TONE)
    {
        audio_set_state(VARIO_AUDIO_CLIMB);
    }
    else
    {
        audio_set_state(VARIO_AUDIO_NEUTRAL);
    }
}

static void vario_task(void *arg)
{
    (void)arg;
    int64_t prev_us = esp_timer_get_time();
    float pressure = 0.0f, temperature = 0.0f;

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
                s_pressure_filt = pressure;
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
            float z = pressure_to_altitude(pressure); // use raw pressure for estimation
            s_last_pressure = pressure;               // store raw last pressure
            s_last_temperature = temperature;
            s_sample_valid = true;
            // Update reporting filter (does not affect estimation)
            float alpha_p = dt / (VARIO_PRESSURE_FILTER_TAU_S + dt);
            s_pressure_filt += alpha_p * (pressure - s_pressure_filt);
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
            // float dt_ms = dt * 1000.0f;
            // float alpha_vs = dt_ms / (VARIO_VS_AUDIO_TAU_MS + dt_ms);
            // s_vspeed += alpha_vs * (s_vspeed_raw - s_vspeed);
            s_vspeed = s_vspeed_raw;

            // TEST: Simulate vertical speed patterns to exercise audio without real sensor motion.
            // Replaces real vspeed audio drive with synthetic profile cycling through sink, neutral, and climb.
            if (conf_test_mode)
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
                    // Climb ramp: +3 -> +5m/s
                    float r = (sim_t - 24.0f) / 4.0f;
                    test_vs = 3.0f + r * 2.0f;
                }
                else if (sim_t < 32.0f)
                {
                    // Hold climb: +5 m/s
                    test_vs = 5.0f;
                }
                else if (sim_t < 36.0f)
                {
                    // Near zero with gentle oscillation: ~ +/-0.5 m/s
                    float r = (sim_t - 32.0f) / 4.0f;
                    test_vs = 0.1f * sinf(r * 2.0f * (float)M_PI);
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

    // 50% duty cycle for push-pull setup
    const uint32_t duty_val = (1U << VARIO_LEDC_DUTY_RES) / 2U;
    // Primary (non-inverted) channel
    ledc_channel_config_t ch_cfg_a = {
        .gpio_num = VARIO_BUZZER_GPIO_A,
        .speed_mode = VARIO_LEDC_MODE,
        .channel = VARIO_LEDC_CHANNEL_A,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = VARIO_LEDC_TIMER,
        .duty = duty_val,
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
        .duty = duty_val,
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

    // Create single state-machine timer
    if (!s_beep_timer)
    {
        const esp_timer_create_args_t targs = {
            .callback = &beep_timer_cb,
            .arg = NULL,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "beep",
        };
        esp_timer_create(&targs, &s_beep_timer);
    }

    // test code for freq response
    // Simple blocking sweep test of buzzer frequency response.
    // Runs once at startup before the vario task begins producing tones.
    // Adjust ranges / step / dwell as desired.
    // {
    //     ESP_LOGI(TAG, "Starting buzzer frequency sweep test");
    // #if VARIO_BUZZER_ACTIVE
    //     // For active buzzer just toggle on/off at a few pseudo "freq" placeholders.
    //     for (int i = 0; i < 5; ++i) {
    //         buzzer_set(1);
    //         vTaskDelay(pdMS_TO_TICKS(300));
    //         buzzer_off();
    //         vTaskDelay(pdMS_TO_TICKS(150));
    //     }
    // #else
    //     const int f_start = 1000;    // Hz
    //     const int f_end   = 3000;   // Hz (limited by code later to 5 kHz)
    //     const int f_step  = 100;    // Hz
    //     const int dwell_ms = 1000;   // time at each frequency
    //     // Up sweep
    //     for (int f = f_start; f <= f_end; f += f_step) {
    //         buzzer_set((float)f);
    //         printf("Buzzer frequency sweep: %d Hz\n", f);
    //         vTaskDelay(pdMS_TO_TICKS(dwell_ms));
    //     }
    //     buzzer_off();
    // #endif
    //     ESP_LOGI(TAG, "Buzzer frequency sweep test complete");
    // }

    gpio_set_drive_capability(VARIO_BUZZER_GPIO_A, GPIO_DRIVE_CAP_3);
    gpio_set_drive_capability(VARIO_BUZZER_GPIO_B, GPIO_DRIVE_CAP_3);
    vTaskDelay(1);
    buzzer_off();

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
    out->pressure_pa = s_pressure_filt;
    out->temperature_c = s_last_temperature;
    out->altitude_m = s_altitude;
    out->vspeed_mps = s_vspeed;
    return true;
}
