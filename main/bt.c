#include "bt.h"
#include <string.h>
#include "esp_log.h"
#include "nvs_flash.h"
#include "host/ble_hs.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#define TAG "BT"

/* Nordic UART Service UUIDs (128-bit) */
static const ble_uuid128_t nus_svc_uuid = BLE_UUID128_INIT(0x9e,0xca,0xdc,0x24,0x0e,0xe5,0xa9,0xe0,0x93,0xf3,0xa3,0xb5,0x00,0x40,0x6e,0xc0);
static const ble_uuid128_t nus_rx_uuid  = BLE_UUID128_INIT(0x9e,0xca,0xdc,0x24,0x0e,0xe5,0xa9,0xe0,0x93,0xf3,0xa3,0xb5,0x01,0x40,0x6e,0xc0);
static const ble_uuid128_t nus_tx_uuid  = BLE_UUID128_INIT(0x9e,0xca,0xdc,0x24,0x0e,0xe5,0xa9,0xe0,0x93,0xf3,0xa3,0xb5,0x02,0x40,0x6e,0xc0);

#define NUS_RX_ATTR_HANDLE 0xFFF1
#define NUS_TX_ATTR_HANDLE 0xFFF2

static uint16_t nus_conn_handle = 0;
static uint16_t nus_tx_val_handle = 0;

static int gatts_nus_access_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg) {
    (void)arg;
    if (attr_handle == nus_tx_val_handle) {
        if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
            /* TX characteristic is Notify only; typically not read. Return empty. */
            ctxt->om = ble_hs_mbuf_from_flat("", 0);
            return 0;
        }
    }
    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR && attr_handle != nus_tx_val_handle) {
        /* Data received from central (RX characteristic) */
        struct os_mbuf *om = ctxt->om;
        uint8_t buf[256];
        int off = 0;
        while (om) {
            int copy = OS_MBUF_PKTLEN(om) - off;
            if (copy > (int)sizeof(buf)) copy = sizeof(buf);
            ble_hs_mbuf_to_flat(om, buf, copy, NULL);
            // For now just log received data
            ESP_LOGI(TAG, "RX: %.*s", copy, buf);
            om = SLIST_NEXT(om, om_next);
        }
        return 0;
    }
    return 0;
}

static const struct ble_gatt_svc_def gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &nus_svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]){
            {
                .uuid = &nus_rx_uuid.u,
                .access_cb = gatts_nus_access_cb,
                .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
            },
            {
                .uuid = &nus_tx_uuid.u,
                .access_cb = gatts_nus_access_cb,
                .val_handle = &nus_tx_val_handle,
                .flags = BLE_GATT_CHR_F_NOTIFY,
            },
            {0} /* no more */
        },
    },
    {0},
};

static int ble_gap_event_cb(struct ble_gap_event *event, void *arg) {
    (void)arg;
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            nus_conn_handle = event->connect.conn_handle;
            ESP_LOGI(TAG, "Connected; conn_handle=%d", nus_conn_handle);
        } else {
            ESP_LOGI(TAG, "Connect failed; retrying advert");
            nus_conn_handle = 0;
            ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER, &(struct ble_gap_adv_params){0}, ble_gap_event_cb, NULL);
        }
        break;
    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "Disconnected; reason=%d", event->disconnect.reason);
        nus_conn_handle = 0;
        ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER, &(struct ble_gap_adv_params){0}, ble_gap_event_cb, NULL);
        break;
    case BLE_GAP_EVENT_ADV_COMPLETE:
        ESP_LOGI(TAG, "Advertising complete; restarting");
        ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER, &(struct ble_gap_adv_params){0}, ble_gap_event_cb, NULL);
        break;
    case BLE_GAP_EVENT_MTU:
        ESP_LOGI(TAG, "MTU update conn=%d mtu=%d", event->mtu.conn_handle, event->mtu.value);
        break;
    default:
        break;
    }
    return 0;
}

static void ble_stack_on_sync(void) {
    /* Use public address (or random static if not present) */
    uint8_t addr_val[6] = {0};
    int rc = ble_hs_id_infer_auto(0, &addr_val[0]);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_hs_id_infer_auto failed: %d", rc);
        return;
    }
    ble_addr_t addr;
    ble_hs_id_copy_addr(BLE_ADDR_PUBLIC, addr_val, NULL);
    addr.type = BLE_ADDR_PUBLIC;

    char addr_str[18];
    sprintf(addr_str, "%02X:%02X:%02X:%02X:%02X:%02X", addr_val[5], addr_val[4], addr_val[3], addr_val[2], addr_val[1], addr_val[0]);
    ESP_LOGI(TAG, "Device Address: %s", addr_str);

    struct ble_hs_adv_fields fields; memset(&fields, 0, sizeof(fields));
    const char *name = "MiniVario";
    fields.name = (uint8_t *)name;
    fields.name_len = strlen(name);
    fields.name_is_complete = 1;
    /* Advertise NUS service UUID */
    fields.uuids128 = (ble_uuid128_t[]){ nus_svc_uuid };
    fields.num_uuids128 = 1;
    fields.uuids128_is_complete = 1;
    ble_gap_adv_set_fields(&fields);

    struct ble_gap_adv_params advp = {0};
    advp.itvl_min = 0x00A0; // 100ms
    advp.itvl_max = 0x00F0; // 150ms
    advp.conn_mode = BLE_GAP_CONN_MODE_UND;
    advp.disc_mode = BLE_GAP_DISC_MODE_GEN;
    ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER, &advp, ble_gap_event_cb, NULL);
}

static void host_task(void *param) {
    nimble_port_run();
    nimble_port_freertos_deinit();
}

bool bt_is_connected(void) { return nus_conn_handle != 0; }

int bt_nus_send(const void *data, uint16_t len) {
    if (!bt_is_connected()) return BLE_HS_ENOTCONN;
    struct os_mbuf *om = ble_hs_mbuf_from_flat(data, len);
    if (!om) return BLE_HS_ENOMEM;
    int rc = ble_gattc_notify_custom(nus_conn_handle, nus_tx_val_handle, om);
    if (rc != 0) {
        ESP_LOGE(TAG, "Notify failed rc=%d", rc);
    }
    return rc;
}

int bt_init(void) {
    int rc;
    /* Initialize NVS needed for BLE */
    rc = nvs_flash_init();
    if (rc != ESP_OK) {
        if (rc == ESP_ERR_NVS_NO_FREE_PAGES || rc == ESP_ERR_NVS_NEW_VERSION_FOUND) {
            nvs_flash_erase();
            rc = nvs_flash_init();
        }
        if (rc != ESP_OK) {
            ESP_LOGE(TAG, "NVS init failed: %d", rc);
            return rc;
        }
    }

    nimble_port_init();

    /* Set device name */
    ble_svc_gap_device_name_set("MiniVario");

    /* GATT services */
    ble_svc_gap_init();
    ble_svc_gatt_init();

    rc = ble_gatts_count_cfg(gatt_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "count_cfg failed: %d", rc); return rc; }
    rc = ble_gatts_add_svcs(gatt_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "add_svcs failed: %d", rc); return rc; }

    ble_hs_cfg.sync_cb = ble_stack_on_sync;

    /* Start host task */
    nimble_port_freertos_init(host_task);

    ESP_LOGI(TAG, "BLE init done");
    return 0;
}
