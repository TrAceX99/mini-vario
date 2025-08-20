// config.h - Runtime configuration flags adjustable via BLE NUS RX
#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

extern volatile bool conf_enable_uart;
extern volatile bool conf_enable_audio;
extern volatile bool conf_send_vario;
extern volatile bool conf_test_mode;
extern volatile bool conf_audio_bt; // when false, audio disabled while BT connected

// Runtime adjustable parameters (persisted in NVS)
extern volatile float conf_kf_accel_std;        // Kalman accel noise std dev (m/s^2)
extern volatile unsigned int conf_inact_timeout_s; // inactivity timeout (s)
extern volatile float conf_climb_min;           // climb tone start threshold (m/s)
extern volatile float conf_climb_max;           // climb tone full-scale threshold (m/s)
extern volatile float conf_sink_min;            // sink tone start threshold (m/s) (absolute value)
extern volatile float conf_sink_max;            // sink tone full-scale threshold (m/s)

bool config_apply_command(const char *cmd, char *out_resp, int resp_size);

// Initialize configuration system (loads persisted values from NVS).
void config_init(void);

// Setters (return previous value). Persist to NVS when changed.
bool  config_set_uart(bool en);
bool  config_set_audio(bool en);
bool  config_set_send_vario(bool en);
bool  config_set_test_mode(bool en);
bool  config_set_audio_bt(bool en);
float config_set_kf_accel_std(float v);
unsigned int config_set_inact_timeout(unsigned int sec);
float config_set_climb_min(float v);
float config_set_climb_max(float v);
float config_set_sink_min(float v);
float config_set_sink_max(float v);

// Format current status into buffer (one line) for GET command.
void config_format_status(char *buffer, int bufsize);


#ifdef __cplusplus
}
#endif
