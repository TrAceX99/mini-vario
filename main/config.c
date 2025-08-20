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
#include "nvs_flash.h"
#include "nvs.h"

volatile bool conf_enable_uart = false;
volatile bool conf_enable_audio = true;
volatile bool conf_send_vario = true;
volatile bool conf_test_mode = false;
volatile bool conf_audio_bt = false; // mute audio when BT connected
volatile float conf_kf_accel_std = 1.5f;
volatile unsigned int conf_inact_timeout_s = 100;
volatile float conf_climb_min = 0.20f;
volatile float conf_climb_max = 5.00f;
volatile float conf_sink_min = 2.00f;
volatile float conf_sink_max = 5.00f;

static nvs_handle_t s_nvs_handle = 0;
static bool s_nvs_ready = false;

void config_format_status(char *buffer, int bufsize)
{
    if (!buffer || bufsize < 8)
        return;
    snprintf(buffer, bufsize, "CFG UART=%d AUDIO=%d AUDIO_BT=%d VARIO=%d KF=%.2f INACT=%u CLIMB=%.2f/%.2f SINK=%.2f/%.2f\n",
             conf_enable_uart?1:0, conf_enable_audio?1:0, conf_audio_bt?1:0, conf_send_vario?1:0,
             conf_kf_accel_std, conf_inact_timeout_s,
             conf_climb_min, conf_climb_max, conf_sink_min, conf_sink_max);
}

void config_init(void)
{
    if (s_nvs_ready) return;
    // Assume nvs_flash_init() already called elsewhere (e.g., BLE stack). Try open directly.
    esp_err_t err = nvs_open("cfg", NVS_READWRITE, &s_nvs_handle);
    if (err == ESP_ERR_NVS_NOT_INITIALIZED) {
        // Fallback if init not done yet
        if (nvs_flash_init() == ESP_OK) {
            err = nvs_open("cfg", NVS_READWRITE, &s_nvs_handle);
        }
    }
    if (err != ESP_OK) {
        printf("CFG NVS open failed err=%d\n", (int)err);
        return;
    }
    uint8_t v;
    if (nvs_get_u8(s_nvs_handle, "uart", &v) == ESP_OK) conf_enable_uart = (v != 0);
    if (nvs_get_u8(s_nvs_handle, "vario", &v) == ESP_OK) conf_send_vario = (v != 0);
    if (nvs_get_u8(s_nvs_handle, "audiobt", &v) == ESP_OK) conf_audio_bt = (v != 0);
    // floats
    float fv;
    if (nvs_get_blob(s_nvs_handle, "kf_accel", &fv, (size_t[]){sizeof(fv)}) == ESP_OK) { if (fv > 0.05f && fv < 10.0f) conf_kf_accel_std = fv; }
    if (nvs_get_blob(s_nvs_handle, "climb_min", &fv, (size_t[]){sizeof(fv)}) == ESP_OK) { conf_climb_min = fv; }
    if (nvs_get_blob(s_nvs_handle, "climb_max", &fv, (size_t[]){sizeof(fv)}) == ESP_OK) { conf_climb_max = fv; }
    if (nvs_get_blob(s_nvs_handle, "sink_min", &fv, (size_t[]){sizeof(fv)}) == ESP_OK) { conf_sink_min = fv; }
    if (nvs_get_blob(s_nvs_handle, "sink_max", &fv, (size_t[]){sizeof(fv)}) == ESP_OK) { conf_sink_max = fv; }
    uint32_t uv;
    if (nvs_get_u32(s_nvs_handle, "inact", &uv) == ESP_OK) { conf_inact_timeout_s = uv; }
    s_nvs_ready = true;
    // print loaded values
    char status[128];
    config_format_status(status, sizeof(status));
    printf("%s", status);
}

bool config_set_uart(bool en)
{
    bool prev = conf_enable_uart;
    conf_enable_uart = en;
    if (prev != en && s_nvs_ready) {
        if (nvs_set_u8(s_nvs_handle, "uart", conf_enable_uart ? 1 : 0) == ESP_OK) {
            nvs_commit(s_nvs_handle);
        }
    }
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
    if (prev != en && s_nvs_ready) {
        if (nvs_set_u8(s_nvs_handle, "vario", conf_send_vario ? 1 : 0) == ESP_OK) {
            nvs_commit(s_nvs_handle);
        }
    }
    return prev;
}

bool config_set_test_mode(bool en)
{
    bool prev = conf_test_mode;
    conf_test_mode = en;
    return prev;
}

bool config_set_audio_bt(bool en)
{
    bool prev = conf_audio_bt;
    conf_audio_bt = en;
    if (prev != en && s_nvs_ready) {
        if (nvs_set_u8(s_nvs_handle, "audiobt", conf_audio_bt ? 1 : 0) == ESP_OK) {
            nvs_commit(s_nvs_handle);
        }
    }
    return prev;
}

static esp_err_t nvs_set_float(const char *key, float v) {
    return nvs_set_blob(s_nvs_handle, key, &v, sizeof(v));
}

float config_set_kf_accel_std(float v)
{
    float prev = conf_kf_accel_std; conf_kf_accel_std = v;
    if (prev != v && s_nvs_ready) { if (nvs_set_float("kf_accel", v)==ESP_OK) nvs_commit(s_nvs_handle);} return prev;
}
unsigned int config_set_inact_timeout(unsigned int sec)
{ unsigned int prev = conf_inact_timeout_s; conf_inact_timeout_s = sec; if (prev!=sec && s_nvs_ready){ if (nvs_set_u32(s_nvs_handle,"inact",sec)==ESP_OK) nvs_commit(s_nvs_handle);} return prev; }
float config_set_climb_min(float v){ float p=conf_climb_min; conf_climb_min=v; if (s_nvs_ready && p!=v){ if (nvs_set_float("climb_min",v)==ESP_OK) nvs_commit(s_nvs_handle);} return p; }
float config_set_climb_max(float v){ float p=conf_climb_max; conf_climb_max=v; if (s_nvs_ready && p!=v){ if (nvs_set_float("climb_max",v)==ESP_OK) nvs_commit(s_nvs_handle);} return p; }
float config_set_sink_min(float v){ float p=conf_sink_min; conf_sink_min=v; if (s_nvs_ready && p!=v){ if (nvs_set_float("sink_min",v)==ESP_OK) nvs_commit(s_nvs_handle);} return p; }
float config_set_sink_max(float v){ float p=conf_sink_max; conf_sink_max=v; if (s_nvs_ready && p!=v){ if (nvs_set_float("sink_max",v)==ESP_OK) nvs_commit(s_nvs_handle);} return p; }

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
        printf("CFG UART=%d AUDIO=%d AUDIO_BT=%d VARIO=%d TEST=%d KF=%.2f INACT=%u CLIMB=%.2f/%.2f SINK=%.2f/%.2f\n", conf_enable_uart ? 1 : 0, conf_enable_audio ? 1 : 0, conf_audio_bt ? 1 : 0, conf_send_vario ? 1 : 0, conf_test_mode ? 1 : 0,
               conf_kf_accel_std, conf_inact_timeout_s, conf_climb_min, conf_climb_max, conf_sink_min, conf_sink_max);

        printf("BATTERY %.0f%% voltage=%.3fV\n", battery_get() * 100.0f, battery_get_voltage());

        return true;
    }
    // Determine if this is a boolean, integer, or float key.
    bool updated = false; bool is_bool_cmd=false;
    int vbool = -1; if (val && (*val=='0' || *val=='1')) { vbool = (*val=='1'); is_bool_cmd=true; }
    if (strcmp(local, "UART") == 0) { if (!is_bool_cmd) goto bad_val; config_set_uart(vbool); updated=true; }
    else if (strcmp(local, "AUDIO") == 0){ if (!is_bool_cmd) goto bad_val; config_set_audio(vbool); updated=true; }
    else if (strcmp(local, "VARIO") == 0){ if (!is_bool_cmd) goto bad_val; config_set_send_vario(vbool); updated=true; }
    else if (strcmp(local, "AUDIO_BT") == 0){ if (!is_bool_cmd) goto bad_val; config_set_audio_bt(vbool); updated=true; }
    else if (strcmp(local, "TEST") == 0){ if (!is_bool_cmd) goto bad_val; config_set_test_mode(vbool); updated=true; }
    else if (strcmp(local, "KF") == 0){ float f=strtof(val,NULL); if (!(f>0.0f)) goto bad_val; config_set_kf_accel_std(f); updated=true; }
    else if (strcmp(local, "INACT") == 0){ int iv=atoi(val); if (iv<=0) goto bad_val; config_set_inact_timeout((unsigned)iv); updated=true; }
    else if (strcmp(local, "CLIMB_MIN") == 0){ float f=strtof(val,NULL); config_set_climb_min(f); updated=true; }
    else if (strcmp(local, "CLIMB_MAX") == 0){ float f=strtof(val,NULL); config_set_climb_max(f); updated=true; }
    else if (strcmp(local, "SINK_MIN") == 0){ float f=strtof(val,NULL); config_set_sink_min(f); updated=true; }
    else if (strcmp(local, "SINK_MAX") == 0){ float f=strtof(val,NULL); config_set_sink_max(f); updated=true; }
    else { if (out_resp) snprintf(out_resp, resp_size, "ERR KEY\n"); return false; }
    if (out_resp && updated)
        config_format_status(out_resp, resp_size);
    return true;
bad_val:
    if (out_resp) snprintf(out_resp, resp_size, "ERR VALUE\n");
    return true;
}
