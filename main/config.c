// config.c - runtime configuration implementation

#include "config.h"

#include <string.h>
#include <stdio.h>
#include <ctype.h>
#include "esp_pm.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "battery.h"

volatile bool conf_enable_uart = false;
volatile bool conf_enable_audio = true;
volatile bool conf_send_vario = true;
volatile bool conf_test_mode = false;

bool config_set_uart(bool en)
{
    bool prev = conf_enable_uart;
    conf_enable_uart = en;
    return prev;
}

bool config_set_audio(bool en)
{
    bool prev = conf_enable_audio;
    conf_enable_audio = en;
    return prev;
}

bool config_set_send_vario(bool en)
{
    bool prev = conf_send_vario;
    conf_send_vario = en;
    return prev;
}

bool config_set_test_mode(bool en)
{
    bool prev = conf_test_mode;
    conf_test_mode = en;
    return prev;
}

void config_format_status(char *buffer, int bufsize)
{
    if (!buffer || bufsize < 8)
        return;
    snprintf(buffer, bufsize, "CFG UART=%d AUDIO=%d VARIO=%d\n", conf_enable_uart ? 1 : 0, conf_enable_audio ? 1 : 0, conf_send_vario ? 1 : 0);
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
    if (strcmp(local, "DEBUG") == 0)
    {
        uint32_t uptime_s = (uint32_t)(esp_timer_get_time() / 1000000ULL);
        size_t free_heap = esp_get_free_heap_size();
        size_t min_free_heap = esp_get_minimum_free_heap_size();
        size_t largest_block = heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT);
        printf("DBG UPTIME=%lus\nHEAP(free=%u min=%u largest=%u)\n", uptime_s, (unsigned)free_heap, (unsigned)min_free_heap, (unsigned)largest_block);

        esp_pm_dump_locks(stdout);

// Task stack high water marks (truncated list)
#if (configUSE_TRACE_FACILITY == 1)
        {
#ifndef MAX_DBG_TASKS
#define MAX_DBG_TASKS 16
#endif
            TaskStatus_t taskStatusArray[MAX_DBG_TASKS];
            UBaseType_t total = uxTaskGetSystemState(taskStatusArray, MAX_DBG_TASKS, NULL);
            printf("TASKS total=%u (showing up to %u)\n", (unsigned)uxTaskGetNumberOfTasks(), (unsigned)total);
            for (UBaseType_t i = 0; i < total; ++i)
            {
                // High water mark is in words (stack depth units); convert to bytes
                unsigned hw = (unsigned)taskStatusArray[i].usStackHighWaterMark * sizeof(StackType_t);
                char state = 'U';
                if (taskStatusArray[i].eCurrentState == eSuspended)
                    state = 'S';
                if (taskStatusArray[i].eCurrentState == eBlocked)
                    state = 'B';
                if (taskStatusArray[i].eCurrentState == eReady)
                    state = 'R';
                if (taskStatusArray[i].eCurrentState == eDeleted)
                    state = 'D';
                if (taskStatusArray[i].eCurrentState == eRunning)
                    state = 'A';
                printf(" T=%s hw=%uB pri=%u s=%c\n", taskStatusArray[i].pcTaskName, hw, (unsigned)taskStatusArray[i].uxCurrentPriority, state);
            }
            if (uxTaskGetNumberOfTasks() > total)
                printf(" ...truncated\n");
        }
#else
        printf("TASK STATS DISABLED (configUSE_TRACE_FACILITY=0)\n");
#endif

        // Current configuration snapshot
    printf("CFG UART=%d AUDIO=%d VARIO=%d TEST=%d\n", conf_enable_uart ? 1 : 0, conf_enable_audio ? 1 : 0, conf_send_vario ? 1 : 0, conf_test_mode ? 1 : 0);

        printf("BATTERY %.0f%% voltage=%.3fV\n", battery_get() * 100.0f, battery_get_voltage());

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
    else if (strcmp(local, "VARIO") == 0)
    {
        config_set_send_vario(v);
        updated = true;
    }
    else if (strcmp(local, "TEST") == 0)
    {
        config_set_test_mode(v);
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
