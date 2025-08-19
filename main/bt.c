#include "bt.h"
#include "config.h"

#include <string.h>
#include "esp_log.h"
#include "nvs_flash.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#define TAG "BT"

/* Nordic UART Service UUIDs (128-bit)
 * Canonical (big-endian):
 *   Service: 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
 *   RX Char: 6E400002-B5A3-F393-E0A9-E50E24DCCA9E
 *   TX Char: 6E400003-B5A3-F393-E0A9-E50E24DCCA9E
 * NimBLE stores uuid128 value[] little-endian (LSB first), so bytes are reversed below.
 */
static const ble_uuid128_t nus_svc_uuid = BLE_UUID128_INIT(0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x00, 0x40, 0x6E);
static const ble_uuid128_t nus_rx_uuid  = BLE_UUID128_INIT(0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x02, 0x00, 0x40, 0x6E);
static const ble_uuid128_t nus_tx_uuid  = BLE_UUID128_INIT(0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x03, 0x00, 0x40, 0x6E);

static const char device_name[] = "MiniVario";

static uint8_t own_addr_type;
static uint8_t addr_val[6] = {0};

static uint16_t nus_conn_handle = 0;
static uint16_t nus_tx_val_handle = 0;
static uint16_t nus_rx_val_handle = 0;
static bool ever_connected = false;

static int ble_gap_event_cb(struct ble_gap_event *event, void *arg);
static int gatts_nus_access_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);

static const struct ble_gatt_svc_def gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &nus_svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]){
            {
                .uuid = &nus_rx_uuid.u,
                .access_cb = gatts_nus_access_cb,
                .val_handle = &nus_rx_val_handle,
                .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
            },
            {
                .uuid = &nus_tx_uuid.u,
                .access_cb = gatts_nus_access_cb,
                .val_handle = &nus_tx_val_handle,
                .flags = BLE_GATT_CHR_F_NOTIFY,
            },
            {0}},
    },
    {0},
};

static void start_advertising(void)
{
    /* Local variables */
    int rc = 0;
    const char *name;
    struct ble_hs_adv_fields adv_fields = {0};
    struct ble_gap_adv_params adv_params = {0};

    /* Set advertising flags */
    adv_fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

    /* Set device name */
    name = ble_svc_gap_device_name();
    adv_fields.name = (uint8_t *)name;
    adv_fields.name_len = strlen(name);
    adv_fields.name_is_complete = 1;

    /* Set advertiement fields */
    rc = ble_gap_adv_set_fields(&adv_fields);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "failed to set advertising data, error code: %d", rc);
        return;
    }

    /* Set non-connectable and general discoverable mode to be a beacon */
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    /* Set advertising interval */
    // adv_params.itvl_min = BLE_GAP_ADV_ITVL_MS(500);
    // adv_params.itvl_max = BLE_GAP_ADV_ITVL_MS(510);

    /* Start advertising */
    rc = ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER, &adv_params, ble_gap_event_cb, NULL);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "failed to start advertising, error code: %d", rc);
        return;
    }
    ESP_LOGI(TAG, "advertising started!");
}

static int ble_gap_event_cb(struct ble_gap_event *event, void *arg)
{
    (void)arg;
    switch (event->type)
    {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0)
        {
            nus_conn_handle = event->connect.conn_handle;
            ever_connected = true;
            ESP_LOGI(TAG, "Connected; conn_handle=%d", nus_conn_handle);
            // print connection descriptor
            struct ble_gap_conn_desc desc;

            ble_gap_conn_find(nus_conn_handle, &desc);
            ESP_LOGI(TAG, "connection handle: %d", desc.conn_handle);

            /* Local ID address */
            ESP_LOGI(TAG, "device id address: type=%d, %02X:%02X:%02X:%02X:%02X:%02X",
                     desc.our_id_addr.type,
                     desc.our_id_addr.val[5], desc.our_id_addr.val[4],
                     desc.our_id_addr.val[3], desc.our_id_addr.val[2],
                     desc.our_id_addr.val[1], desc.our_id_addr.val[0]);

            /* Peer ID address */
            ESP_LOGI(TAG, "peer id address: type=%d, %02X:%02X:%02X:%02X:%02X:%02X",
                     desc.peer_id_addr.type,
                     desc.peer_id_addr.val[5], desc.peer_id_addr.val[4],
                     desc.peer_id_addr.val[3], desc.peer_id_addr.val[2],
                     desc.peer_id_addr.val[1], desc.peer_id_addr.val[0]);

            /* Connection info */
            ESP_LOGI(TAG,
                     "conn_itvl=%d, conn_latency=%d, supervision_timeout=%d, "
                     "encrypted=%d, authenticated=%d, bonded=%d\n",
                     desc.conn_itvl, desc.conn_latency, desc.supervision_timeout,
                     desc.sec_state.encrypted, desc.sec_state.authenticated,
                     desc.sec_state.bonded);
        }
        else
        {
            ESP_LOGI(TAG, "Connect failed; retrying advert");
            nus_conn_handle = 0;
            start_advertising();
        }
        break;
    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "Disconnected; reason=%d", event->disconnect.reason);
        nus_conn_handle = 0;
        start_advertising();
        break;
    case BLE_GAP_EVENT_ADV_COMPLETE:
        ESP_LOGI(TAG, "Advertising complete; restarting");
        start_advertising();
        break;
    default:
        break;
    }
    return 0;
}

static int gatts_nus_access_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)arg;
    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR && attr_handle == nus_rx_val_handle) {
        uint8_t buf[128];
        uint16_t copied = 0;
        ble_hs_mbuf_to_flat(ctxt->om, buf, sizeof(buf)-1, &copied);
        buf[copied] = '\0';
        ESP_LOGI(TAG, "RX CMD: %s", buf);
        char resp[64] = {0};
        if (config_apply_command((const char*)buf, resp, sizeof(resp))) {
            if (resp[0]) {
                bt_nus_send(resp, (uint16_t)strlen(resp));
            }
        } else {
            const char *err = "ERR\n";
            bt_nus_send(err, (uint16_t)strlen(err));
        }
        return 0;
    }
    return 0;
}

static void ble_stack_on_sync(void)
{
    /* Local variables */
    int rc = 0;

    /* Figure out BT address to use while advertising */
    // rc = ble_hs_id_infer_auto(0, &own_addr_type);
    own_addr_type = BLE_OWN_ADDR_PUBLIC;
    if (rc != 0)
    {
        ESP_LOGE(TAG, "failed to infer address type, error code: %d", rc);
        return;
    }

    /* Printing ADDR */
    rc = ble_hs_id_copy_addr(own_addr_type, addr_val, NULL);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "failed to copy device address, error code: %d", rc);
        return;
    }
    ESP_LOGI(TAG, "device address: %02X:%02X:%02X:%02X:%02X:%02X",
             addr_val[5], addr_val[4], addr_val[3],
             addr_val[2], addr_val[1], addr_val[0]);

    start_advertising();
}

static void host_task(void *param)
{
    nimble_port_run();
    nimble_port_freertos_deinit();
}

bool bt_is_connected(void) { return nus_conn_handle != 0; }
bool bt_has_ever_connected(void) { return ever_connected; }

int bt_nus_send(const void *data, uint16_t len)
{
    if (!bt_is_connected())
    {
        return BLE_HS_ENOTCONN;
    }
    struct os_mbuf *om = ble_hs_mbuf_from_flat(data, len);
    if (!om)
    {
        return BLE_HS_ENOMEM;
    }
    int rc = ble_gatts_notify_custom(nus_conn_handle, nus_tx_val_handle, om);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "Notify failed rc=%d", rc);
    }
    return rc;
}

int bt_init(void)
{
    int rc;
    /* Initialize NVS needed for BLE */
    rc = nvs_flash_init();
    if (rc != ESP_OK)
    {
        if (rc == ESP_ERR_NVS_NO_FREE_PAGES || rc == ESP_ERR_NVS_NEW_VERSION_FOUND)
        {
            ESP_ERROR_CHECK(nvs_flash_erase());
            rc = nvs_flash_init();
        }
        if (rc != ESP_OK)
        {
            ESP_LOGE(TAG, "NVS init failed: %d", rc);
            return rc;
        }
    }

    /* Stack init */
    nimble_port_init();

    /* GAP init */
    ble_svc_gap_init();
    ble_svc_gap_device_name_set(device_name);

    /* GATT services */
    ble_svc_gatt_init();
    rc = ble_gatts_count_cfg(gatt_svcs);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "count_cfg failed: %d", rc);
        return rc;
    }
    rc = ble_gatts_add_svcs(gatt_svcs);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "add_svcs failed: %d", rc);
        return rc;
    }

    ble_hs_cfg.sync_cb = ble_stack_on_sync;

    /* Start host task */
    nimble_port_freertos_init(host_task);

    ESP_LOGI(TAG, "BLE init done");
    return 0;
}

int bt_deinit(void)
{
    ESP_LOGI(TAG, "Shutting down BLE stack");
    // Stop advertising (ignore errors)
    ble_gap_adv_stop();
    // Disconnect if connected
    if (nus_conn_handle != 0) {
        ble_gap_terminate(nus_conn_handle, BLE_ERR_REM_USER_CONN_TERM);
        nus_conn_handle = 0;
    }
    // Stop NimBLE host task
    nimble_port_stop();
    nimble_port_freertos_deinit();
    nimble_port_deinit();
    ESP_LOGI(TAG, "BLE stack shutdown complete");
    return 0;
}
