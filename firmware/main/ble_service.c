#include "ble_service.h"
#include "imu_task.h"
#include "protocol.h"

#include "esp_log.h"
#include "esp_nimble_hci.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

static const char *TAG = "ble_service";

#define DEVICE_NAME "BarbellIMU"

// Connection handle
static uint16_t s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static bool s_notifications_enabled = false;

// Characteristic value handles
static uint16_t s_stream_val_handle;
static uint16_t s_status_val_handle;

// UUIDs
static const ble_uuid128_t s_service_uuid = BLE_UUID128_INIT(BARBELL_IMU_SERVICE_UUID);
static const ble_uuid128_t s_stream_uuid = BLE_UUID128_INIT(BARBELL_IMU_STREAM_CHAR_UUID);
static const ble_uuid128_t s_command_uuid = BLE_UUID128_INIT(BARBELL_IMU_COMMAND_CHAR_UUID);
static const ble_uuid128_t s_status_uuid = BLE_UUID128_INIT(BARBELL_IMU_STATUS_CHAR_UUID);

// Status buffer
static status_payload_t s_status_buf;

// Forward declarations
static int stream_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                           struct ble_gatt_access_ctxt *ctxt, void *arg);
static int command_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                            struct ble_gatt_access_ctxt *ctxt, void *arg);
static int status_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                           struct ble_gatt_access_ctxt *ctxt, void *arg);
static int gap_event_cb(struct ble_gap_event *event, void *arg);

// GATT service definition
static const struct ble_gatt_svc_def s_gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &s_service_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                // Stream characteristic (notify)
                .uuid = &s_stream_uuid.u,
                .access_cb = stream_access_cb,
                .val_handle = &s_stream_val_handle,
                .flags = BLE_GATT_CHR_F_NOTIFY,
            },
            {
                // Command characteristic (write)
                .uuid = &s_command_uuid.u,
                .access_cb = command_access_cb,
                .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
            },
            {
                // Status characteristic (read, notify)
                .uuid = &s_status_uuid.u,
                .access_cb = status_access_cb,
                .val_handle = &s_status_val_handle,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
            },
            {0}, // Terminator
        },
    },
    {0}, // Terminator
};

static int stream_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                           struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    // Stream is notify-only, no read access
    return BLE_ATT_ERR_READ_NOT_PERMITTED;
}

static int command_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                            struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) {
        return BLE_ATT_ERR_UNLIKELY;
    }

    uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
    if (len < 1) {
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }

    uint8_t cmd_buf[8];
    uint16_t copy_len = len > sizeof(cmd_buf) ? sizeof(cmd_buf) : len;
    os_mbuf_copydata(ctxt->om, 0, copy_len, cmd_buf);

    uint8_t opcode = cmd_buf[0];

    switch (opcode) {
        case CMD_START_STREAM: {
            uint16_t rate_hz = DEFAULT_STREAM_RATE_HZ;
            if (len >= 3) {
                rate_hz = cmd_buf[1] | (cmd_buf[2] << 8);
            }
            ESP_LOGI(TAG, "CMD: START_STREAM rate=%d Hz", rate_hz);
            imu_start_stream(rate_hz);
            break;
        }

        case CMD_STOP_STREAM:
            ESP_LOGI(TAG, "CMD: STOP_STREAM");
            imu_stop_stream();
            break;

        case CMD_CALIBRATE:
            ESP_LOGI(TAG, "CMD: CALIBRATE");
            imu_calibrate();
            break;

        case CMD_SET_CONFIG:
            ESP_LOGI(TAG, "CMD: SET_CONFIG (not implemented in MVP)");
            break;

        default:
            ESP_LOGW(TAG, "Unknown command: 0x%02X", opcode);
            return BLE_ATT_ERR_REQ_NOT_SUPPORTED;
    }

    return 0;
}

static int status_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                           struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        imu_get_status(&s_status_buf);
        int rc = os_mbuf_append(ctxt->om, &s_status_buf, sizeof(s_status_buf));
        return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }

    return BLE_ATT_ERR_UNLIKELY;
}

static void on_sync(void)
{
    int rc;

    // Use public address
    rc = ble_hs_util_ensure_addr(0);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_hs_util_ensure_addr failed: %d", rc);
        return;
    }

    // Start advertising
    struct ble_gap_adv_params adv_params = {0};
    struct ble_hs_adv_fields fields = {0};

    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.tx_pwr_lvl_is_present = 1;
    fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;
    fields.name = (uint8_t *)DEVICE_NAME;
    fields.name_len = strlen(DEVICE_NAME);
    fields.name_is_complete = 1;

    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_adv_set_fields failed: %d", rc);
        return;
    }

    // Set scan response with service UUID
    struct ble_hs_adv_fields rsp_fields = {0};
    rsp_fields.uuids128 = (ble_uuid128_t[]){s_service_uuid};
    rsp_fields.num_uuids128 = 1;
    rsp_fields.uuids128_is_complete = 1;

    rc = ble_gap_adv_rsp_set_fields(&rsp_fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_adv_rsp_set_fields failed: %d", rc);
        return;
    }

    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    rc = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER,
                           &adv_params, gap_event_cb, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_adv_start failed: %d", rc);
        return;
    }

    ESP_LOGI(TAG, "Advertising started");
}

static void on_reset(int reason)
{
    ESP_LOGW(TAG, "BLE reset, reason=%d", reason);
}

static int gap_event_cb(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT:
            if (event->connect.status == 0) {
                s_conn_handle = event->connect.conn_handle;
                ESP_LOGI(TAG, "Client connected, handle=%d", s_conn_handle);
            } else {
                ESP_LOGW(TAG, "Connection failed, status=%d", event->connect.status);
                // Restart advertising
                on_sync();
            }
            break;

        case BLE_GAP_EVENT_DISCONNECT:
            ESP_LOGI(TAG, "Client disconnected, reason=%d", event->disconnect.reason);
            s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
            s_notifications_enabled = false;
            imu_stop_stream();  // Stop streaming on disconnect
            // Restart advertising
            on_sync();
            break;

        case BLE_GAP_EVENT_SUBSCRIBE:
            if (event->subscribe.attr_handle == s_stream_val_handle) {
                s_notifications_enabled = event->subscribe.cur_notify;
                ESP_LOGI(TAG, "Stream notifications %s",
                         s_notifications_enabled ? "enabled" : "disabled");
            }
            break;

        case BLE_GAP_EVENT_MTU:
            ESP_LOGI(TAG, "MTU updated: %d", event->mtu.value);
            break;

        default:
            break;
    }

    return 0;
}

static void ble_host_task(void *param)
{
    ESP_LOGI(TAG, "BLE host task started");
    nimble_port_run();
    nimble_port_freertos_deinit();
}

void ble_service_init(void)
{
    int rc;

    // Initialize NimBLE
    rc = nimble_port_init();
    if (rc != ESP_OK) {
        ESP_LOGE(TAG, "nimble_port_init failed: %d", rc);
        return;
    }

    // Configure host callbacks
    ble_hs_cfg.reset_cb = on_reset;
    ble_hs_cfg.sync_cb = on_sync;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    // Initialize GAP and GATT services
    ble_svc_gap_init();
    ble_svc_gatt_init();

    // Register custom service
    rc = ble_gatts_count_cfg(s_gatt_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gatts_count_cfg failed: %d", rc);
        return;
    }

    rc = ble_gatts_add_svcs(s_gatt_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gatts_add_svcs failed: %d", rc);
        return;
    }

    // Set device name
    rc = ble_svc_gap_device_name_set(DEVICE_NAME);
    if (rc != 0) {
        ESP_LOGW(TAG, "Failed to set device name: %d", rc);
    }

    // Start host task
    nimble_port_freertos_init(ble_host_task);

    ESP_LOGI(TAG, "BLE service initialized");
}

bool ble_is_connected(void)
{
    return s_conn_handle != BLE_HS_CONN_HANDLE_NONE;
}

bool ble_notify_stream(const void *data, size_t len)
{
    if (!ble_is_connected() || !s_notifications_enabled) {
        return false;
    }

    struct os_mbuf *om = ble_hs_mbuf_from_flat(data, len);
    if (!om) {
        return false;
    }

    int rc = ble_gatts_notify_custom(s_conn_handle, s_stream_val_handle, om);
    if (rc != 0) {
        ESP_LOGW(TAG, "Notify failed: %d", rc);
        return false;
    }

    return true;
}

void ble_update_status(const void *data, size_t len)
{
    if (!ble_is_connected()) {
        return;
    }

    struct os_mbuf *om = ble_hs_mbuf_from_flat(data, len);
    if (om) {
        ble_gatts_notify_custom(s_conn_handle, s_status_val_handle, om);
    }
}
