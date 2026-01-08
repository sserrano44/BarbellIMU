#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "protocol.h"

// Default configuration
#define DEFAULT_STREAM_RATE_HZ  100
#define MIN_STREAM_RATE_HZ      50
#define MAX_STREAM_RATE_HZ      200

// Calibration samples
#define CALIBRATION_SAMPLES     500

// GPIO pins for I2C (ESP32-C3)
#define I2C_SDA_PIN             8
#define I2C_SCL_PIN             9

/**
 * @brief Initialize IMU task
 */
void imu_task_init(void);

/**
 * @brief Start streaming at given rate
 * @param rate_hz Stream rate in Hz (clamped to MIN/MAX)
 */
void imu_start_stream(uint16_t rate_hz);

/**
 * @brief Stop streaming
 */
void imu_stop_stream(void);

/**
 * @brief Start gyro calibration
 * Collects samples while stationary to compute bias
 */
void imu_calibrate(void);

/**
 * @brief Check if currently streaming
 */
bool imu_is_streaming(void);

/**
 * @brief Check if calibrated
 */
bool imu_is_calibrated(void);

/**
 * @brief Get current stream rate
 */
uint16_t imu_get_stream_rate(void);

/**
 * @brief Get latest packet (for BLE notification)
 * @param packet Pointer to receive packet
 * @return true if new packet available
 */
bool imu_get_packet(stream_packet_t *packet);

/**
 * @brief Get current status
 */
void imu_get_status(status_payload_t *status);
