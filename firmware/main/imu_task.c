#include "imu_task.h"
#include "mpu6050.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <string.h>
#include <math.h>

static const char *TAG = "imu_task";

// IMU handle
static mpu6050_handle_t s_mpu = NULL;

// State
static volatile bool s_streaming = false;
static volatile bool s_calibrated = false;
static volatile bool s_calibrating = false;
static volatile uint16_t s_stream_rate_hz = DEFAULT_STREAM_RATE_HZ;

// Calibration offsets (raw counts)
static int32_t s_gyro_offset_x = 0;
static int32_t s_gyro_offset_y = 0;
static int32_t s_gyro_offset_z = 0;

// Latest packet
static stream_packet_t s_packet;
static volatile bool s_packet_ready = false;
static SemaphoreHandle_t s_packet_mutex = NULL;

// Sequence counter
static uint8_t s_seq = 0;

// Sensitivities
static float s_accel_sens = 2048.0f;  // LSB/g for ±16g
static float s_gyro_sens = 16.4f;     // LSB/(deg/s) for ±2000dps

// Task handle
static TaskHandle_t s_imu_task_handle = NULL;

// Forward declaration
static void imu_sampling_task(void *arg);

void imu_task_init(void)
{
    // Create mutex
    s_packet_mutex = xSemaphoreCreateMutex();
    if (!s_packet_mutex) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return;
    }

    // Configure MPU6050
    mpu6050_config_t cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_pin = I2C_SDA_PIN,
        .scl_pin = I2C_SCL_PIN,
        .i2c_freq = 400000,  // 400kHz I2C
        .accel_fs = MPU6050_ACCEL_FS_16G,
        .gyro_fs = MPU6050_GYRO_FS_2000,
        .dlpf = MPU6050_DLPF_44HZ,
        .sample_rate_div = 4,  // 1kHz / 5 = 200Hz internal
    };

    esp_err_t ret = mpu6050_init(&cfg, &s_mpu);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "MPU6050 init failed: %s", esp_err_to_name(ret));
        return;
    }

    s_accel_sens = mpu6050_get_accel_sensitivity(s_mpu);
    s_gyro_sens = mpu6050_get_gyro_sensitivity(s_mpu);

    ESP_LOGI(TAG, "MPU6050 ready. Accel sens: %.1f LSB/g, Gyro sens: %.2f LSB/(deg/s)",
             s_accel_sens, s_gyro_sens);

    // Create sampling task (high priority)
    xTaskCreatePinnedToCore(
        imu_sampling_task,
        "imu_task",
        4096,
        NULL,
        configMAX_PRIORITIES - 1,
        &s_imu_task_handle,
        tskNO_AFFINITY  // Any core (ESP32-C3 is single-core)
    );
}

static void imu_sampling_task(void *arg)
{
    mpu6050_raw_data_t raw;
    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
        uint32_t interval_ms = 1000 / s_stream_rate_hz;
        if (interval_ms < 1) interval_ms = 1;

        if (s_calibrating) {
            // Calibration mode: collect samples
            int32_t sum_gx = 0, sum_gy = 0, sum_gz = 0;
            int valid_samples = 0;

            ESP_LOGI(TAG, "Starting calibration...");
            vTaskDelay(pdMS_TO_TICKS(500));  // Let sensor settle

            for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
                if (mpu6050_read_raw(s_mpu, &raw) == ESP_OK) {
                    sum_gx += raw.gyro_x;
                    sum_gy += raw.gyro_y;
                    sum_gz += raw.gyro_z;
                    valid_samples++;
                }
                vTaskDelay(pdMS_TO_TICKS(2));
            }

            if (valid_samples > 0) {
                s_gyro_offset_x = sum_gx / valid_samples;
                s_gyro_offset_y = sum_gy / valid_samples;
                s_gyro_offset_z = sum_gz / valid_samples;
                s_calibrated = true;
                ESP_LOGI(TAG, "Calibration complete. Offsets: gx=%ld, gy=%ld, gz=%ld",
                         s_gyro_offset_x, s_gyro_offset_y, s_gyro_offset_z);
            } else {
                ESP_LOGE(TAG, "Calibration failed - no valid samples");
            }

            s_calibrating = false;

        } else if (s_streaming) {
            // Streaming mode
            if (mpu6050_read_raw(s_mpu, &raw) == ESP_OK) {
                uint32_t now_us = (uint32_t)esp_timer_get_time();

                // Apply gyro calibration
                int16_t cal_gx = raw.gyro_x - (int16_t)s_gyro_offset_x;
                int16_t cal_gy = raw.gyro_y - (int16_t)s_gyro_offset_y;
                int16_t cal_gz = raw.gyro_z - (int16_t)s_gyro_offset_z;

                // Convert to packet units
                // Accel: raw to milli-g = raw * 1000 / sensitivity
                // Gyro: raw to (deg/s)*16 = raw * 16 / sensitivity
                int16_t ax_mg = (int16_t)((raw.accel_x * 1000) / (int32_t)s_accel_sens);
                int16_t ay_mg = (int16_t)((raw.accel_y * 1000) / (int32_t)s_accel_sens);
                int16_t az_mg = (int16_t)((raw.accel_z * 1000) / (int32_t)s_accel_sens);

                int16_t gx_dps16 = (int16_t)((cal_gx * 16) / (int32_t)s_gyro_sens);
                int16_t gy_dps16 = (int16_t)((cal_gy * 16) / (int32_t)s_gyro_sens);
                int16_t gz_dps16 = (int16_t)((cal_gz * 16) / (int32_t)s_gyro_sens);

                // Build packet
                stream_packet_t pkt = {
                    .timestamp_us = now_us,
                    .ax_mg = ax_mg,
                    .ay_mg = ay_mg,
                    .az_mg = az_mg,
                    .gx_dps16 = gx_dps16,
                    .gy_dps16 = gy_dps16,
                    .gz_dps16 = gz_dps16,
                    .seq = s_seq++,
                    .flags = (s_streaming ? FLAG_STREAMING : 0) | (s_calibrated ? FLAG_CALIBRATED : 0),
                    .reserved = 0,
                };

                // Store packet
                if (xSemaphoreTake(s_packet_mutex, 0) == pdTRUE) {
                    memcpy(&s_packet, &pkt, sizeof(pkt));
                    s_packet_ready = true;
                    xSemaphoreGive(s_packet_mutex);
                }
            }

            vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(interval_ms));
        } else {
            // Idle - wait for command
            vTaskDelay(pdMS_TO_TICKS(100));
            last_wake = xTaskGetTickCount();
        }
    }
}

void imu_start_stream(uint16_t rate_hz)
{
    // Clamp rate
    if (rate_hz < MIN_STREAM_RATE_HZ) rate_hz = MIN_STREAM_RATE_HZ;
    if (rate_hz > MAX_STREAM_RATE_HZ) rate_hz = MAX_STREAM_RATE_HZ;

    s_stream_rate_hz = rate_hz;
    s_seq = 0;
    s_streaming = true;

    ESP_LOGI(TAG, "Streaming started at %d Hz", rate_hz);
}

void imu_stop_stream(void)
{
    s_streaming = false;
    ESP_LOGI(TAG, "Streaming stopped");
}

void imu_calibrate(void)
{
    s_streaming = false;
    s_calibrating = true;
    ESP_LOGI(TAG, "Calibration requested");
}

bool imu_is_streaming(void)
{
    return s_streaming;
}

bool imu_is_calibrated(void)
{
    return s_calibrated;
}

uint16_t imu_get_stream_rate(void)
{
    return s_stream_rate_hz;
}

bool imu_get_packet(stream_packet_t *packet)
{
    if (!s_packet_ready) {
        return false;
    }

    if (xSemaphoreTake(s_packet_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        if (s_packet_ready) {
            memcpy(packet, &s_packet, sizeof(stream_packet_t));
            s_packet_ready = false;
            xSemaphoreGive(s_packet_mutex);
            return true;
        }
        xSemaphoreGive(s_packet_mutex);
    }
    return false;
}

void imu_get_status(status_payload_t *status)
{
    status->stream_rate_hz = s_stream_rate_hz;
    status->accel_fs = MPU6050_ACCEL_FS_16G;
    status->gyro_fs = MPU6050_GYRO_FS_2000;
    status->dlpf = MPU6050_DLPF_44HZ;
    status->flags = (s_streaming ? FLAG_STREAMING : 0) | (s_calibrated ? FLAG_CALIBRATED : 0);
    status->fw_version_major = FW_VERSION_MAJOR;
    status->fw_version_minor = FW_VERSION_MINOR;
}
