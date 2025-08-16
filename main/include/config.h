// config.h - Runtime configuration flags adjustable via BLE NUS RX
#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

extern volatile bool conf_enable_uart;
extern volatile bool conf_enable_audio;
extern volatile bool conf_enable_bluetooth;

bool config_set_uart(bool en);
bool config_set_audio(bool en);
bool config_set_bluetooth(bool en);

void config_format_status(char *buffer, int bufsize);
bool config_apply_command(const char *cmd, char *out_resp, int resp_size);

#ifdef __cplusplus
}
#endif
