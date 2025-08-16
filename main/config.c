// config.c - runtime configuration implementation

#include "config.h"

#include <string.h>
#include <stdio.h>
#include <ctype.h>
#include "esp_pm.h"

volatile bool conf_enable_uart = true;
volatile bool conf_enable_audio = true;
volatile bool conf_enable_bluetooth = true;
volatile bool conf_send_vario = true;

bool config_set_uart(bool en)
{
    bool prev = conf_enable_uart;
    conf_enable_uart = en;
    esp_pm_config_t pm_config = {
        .max_freq_mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ,
        .min_freq_mhz = CONFIG_XTAL_FREQ,
        .light_sleep_enable = !en,
    };
    esp_pm_configure(&pm_config);
    return prev;
}
bool config_set_audio(bool en)
{
    bool prev = conf_enable_audio;
    conf_enable_audio = en;
    return prev;
}
bool config_set_bluetooth(bool en)
{
    bool prev = conf_enable_bluetooth;
    conf_enable_bluetooth = en;
    return prev;
}
bool config_set_send_vario(bool en)
{
    bool prev = conf_send_vario;
    conf_send_vario = en;
    return prev;
}

void config_format_status(char *buffer, int bufsize)
{
    if (!buffer || bufsize < 8)
        return;
    snprintf(buffer, bufsize, "CFG UART=%d AUDIO=%d BLE=%d VARIO=%d\n", conf_enable_uart ? 1 : 0, conf_enable_audio ? 1 : 0, conf_enable_bluetooth ? 1 : 0, conf_send_vario ? 1 : 0);
}

static void str_toupper(char *s)
{
    while (s && *s)
    {
        *s = (char)toupper((unsigned char)*s);
        ++s;
    }
}

bool config_apply_command(const char *cmd_in, char *out_resp, int resp_size)
{
    if (!cmd_in)
        return false;
    char local[64];
    snprintf(local, sizeof(local), "%s", cmd_in);
    for (char *p = local; *p; ++p)
    {
        if (*p == '\r' || *p == '\n')
        {
            *p = '\0';
            break;
        }
    }
    char *eq = strchr(local, '=');
    if (eq)
        *eq = '\0';
    char *val = eq ? eq + 1 : "";
    str_toupper(local);
    if (strcmp(local, "GET") == 0)
    {
        if (out_resp)
            config_format_status(out_resp, resp_size);
        return true;
    }
    int v = -1;
    if (val && *val)
        v = (*val == '0') ? 0 : ((*val == '1') ? 1 : -1);
    if (v < 0)
    {
        if (out_resp)
            snprintf(out_resp, resp_size, "ERR VALUE\n");
        return false;
    }
    bool updated = false;
    if (strcmp(local, "UART") == 0)
    {
        config_set_uart(v);
        updated = true;
    }
    else if (strcmp(local, "AUDIO") == 0)
    {
        config_set_audio(v);
        updated = true;
    }
    else if (strcmp(local, "BLE") == 0)
    {
        config_set_bluetooth(v);
        updated = true;
    }
    else if (strcmp(local, "VARIO") == 0)
    {
        config_set_send_vario(v);
        updated = true;
    }
    else
    {
        if (out_resp)
            snprintf(out_resp, resp_size, "ERR KEY\n");
        return false;
    }
    if (out_resp && updated)
        config_format_status(out_resp, resp_size);
    return true;
}
