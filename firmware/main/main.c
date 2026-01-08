#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_log.h"

#include "ble_service.h"
#include "imu_task.h"
#include "protocol.h"

static const char *TAG = "main";

static void stream_task(void *arg)
{
    stream_packet_t packet;

    ESP_LOGI(TAG, "Stream notification task started");

    while (1) {
        if (imu_is_streaming() && ble_is_connected()) {
            if (imu_get_packet(&packet)) {
                ble_notify_stream(&packet, sizeof(packet));
            }
            // Check frequently when streaming
            vTaskDelay(pdMS_TO_TICKS(1));
        } else {
            // Idle - check less often
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Barbell IMU v%d.%d starting...", FW_VERSION_MAJOR, FW_VERSION_MINOR);

    // Initialize NVS (required for BLE)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize IMU
    imu_task_init();

    // Initialize BLE
    ble_service_init();

    // Create task to send BLE notifications
    xTaskCreatePinnedToCore(
        stream_task,
        "stream_task",
        2048,
        NULL,
        5,
        NULL,
        tskNO_AFFINITY  // Any core (ESP32-C3 is single-core)
    );

    ESP_LOGI(TAG, "Initialization complete. Waiting for BLE connection...");
}
