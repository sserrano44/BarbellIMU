#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c.h"

#define MPU6050_ADDR 0x68

// Accel full-scale range
typedef enum {
    MPU6050_ACCEL_FS_2G  = 0,
    MPU6050_ACCEL_FS_4G  = 1,
    MPU6050_ACCEL_FS_8G  = 2,
    MPU6050_ACCEL_FS_16G = 3,
} mpu6050_accel_fs_t;

// Gyro full-scale range
typedef enum {
    MPU6050_GYRO_FS_250  = 0,
    MPU6050_GYRO_FS_500  = 1,
    MPU6050_GYRO_FS_1000 = 2,
    MPU6050_GYRO_FS_2000 = 3,
} mpu6050_gyro_fs_t;

// DLPF bandwidth settings
typedef enum {
    MPU6050_DLPF_260HZ = 0,
    MPU6050_DLPF_184HZ = 1,
    MPU6050_DLPF_94HZ  = 2,
    MPU6050_DLPF_44HZ  = 3,
    MPU6050_DLPF_21HZ  = 4,
    MPU6050_DLPF_10HZ  = 5,
    MPU6050_DLPF_5HZ   = 6,
} mpu6050_dlpf_t;

// Raw sensor data
typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    int16_t temp;
} mpu6050_raw_data_t;

// Configuration
typedef struct {
    i2c_port_t i2c_port;
    int sda_pin;
    int scl_pin;
    uint32_t i2c_freq;
    mpu6050_accel_fs_t accel_fs;
    mpu6050_gyro_fs_t gyro_fs;
    mpu6050_dlpf_t dlpf;
    uint8_t sample_rate_div;  // Sample rate = 1kHz / (1 + div) when DLPF enabled
} mpu6050_config_t;

// Handle
typedef struct mpu6050_dev_t* mpu6050_handle_t;

/**
 * @brief Initialize MPU6050 with given configuration
 */
esp_err_t mpu6050_init(const mpu6050_config_t *config, mpu6050_handle_t *handle);

/**
 * @brief Deinitialize MPU6050
 */
esp_err_t mpu6050_deinit(mpu6050_handle_t handle);

/**
 * @brief Read raw sensor data
 */
esp_err_t mpu6050_read_raw(mpu6050_handle_t handle, mpu6050_raw_data_t *data);

/**
 * @brief Get accel sensitivity in LSB/g
 */
float mpu6050_get_accel_sensitivity(mpu6050_handle_t handle);

/**
 * @brief Get gyro sensitivity in LSB/(deg/s)
 */
float mpu6050_get_gyro_sensitivity(mpu6050_handle_t handle);

/**
 * @brief Check if device is connected and responding
 */
esp_err_t mpu6050_test_connection(mpu6050_handle_t handle);
