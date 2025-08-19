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

bool config_apply_command(const char *cmd, char *out_resp, int resp_size);

#ifdef __cplusplus
}
#endif
