#include "mpu6050.h"
#include <stdlib.h>
#include <string.h>
#include "esp_log.h"

static const char *TAG = "mpu6050";

// MPU6050 Register addresses
#define MPU6050_REG_SMPLRT_DIV    0x19
#define MPU6050_REG_CONFIG        0x1A
#define MPU6050_REG_GYRO_CONFIG   0x1B
#define MPU6050_REG_ACCEL_CONFIG  0x1C
#define MPU6050_REG_ACCEL_XOUT_H  0x3B
#define MPU6050_REG_TEMP_OUT_H    0x41
#define MPU6050_REG_GYRO_XOUT_H   0x43
#define MPU6050_REG_PWR_MGMT_1    0x6B
#define MPU6050_REG_PWR_MGMT_2    0x6C
#define MPU6050_REG_WHO_AM_I      0x75

#define MPU6050_WHO_AM_I_VAL      0x68

// Internal device structure
struct mpu6050_dev_t {
    i2c_port_t i2c_port;
    mpu6050_accel_fs_t accel_fs;
    mpu6050_gyro_fs_t gyro_fs;
};

static esp_err_t mpu6050_write_reg(mpu6050_handle_t handle, uint8_t reg, uint8_t data)
{
    uint8_t buf[2] = {reg, data};
    return i2c_master_write_to_device(handle->i2c_port, MPU6050_ADDR, buf, 2, pdMS_TO_TICKS(100));
}

static esp_err_t mpu6050_read_reg(mpu6050_handle_t handle, uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(handle->i2c_port, MPU6050_ADDR, &reg, 1, data, len, pdMS_TO_TICKS(100));
}

esp_err_t mpu6050_init(const mpu6050_config_t *config, mpu6050_handle_t *handle)
{
    if (!config || !handle) {
        return ESP_ERR_INVALID_ARG;
    }

    // Initialize I2C
    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = config->sda_pin,
        .scl_io_num = config->scl_pin,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = config->i2c_freq,
    };

    esp_err_t ret = i2c_param_config(config->i2c_port, &i2c_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C param config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = i2c_driver_install(config->i2c_port, I2C_MODE_MASTER, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver install failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Allocate handle
    struct mpu6050_dev_t *dev = calloc(1, sizeof(struct mpu6050_dev_t));
    if (!dev) {
        i2c_driver_delete(config->i2c_port);
        return ESP_ERR_NO_MEM;
    }

    dev->i2c_port = config->i2c_port;
    dev->accel_fs = config->accel_fs;
    dev->gyro_fs = config->gyro_fs;

    *handle = dev;

    // Wake up MPU6050 (clear sleep bit, use internal 8MHz oscillator)
    ret = mpu6050_write_reg(dev, MPU6050_REG_PWR_MGMT_1, 0x00);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to wake MPU6050");
        goto err;
    }

    vTaskDelay(pdMS_TO_TICKS(100));  // Wait for wake-up

    // Verify connection
    ret = mpu6050_test_connection(dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "MPU6050 not responding");
        goto err;
    }

    // Configure DLPF
    ret = mpu6050_write_reg(dev, MPU6050_REG_CONFIG, config->dlpf);
    if (ret != ESP_OK) goto err;

    // Configure sample rate divider
    ret = mpu6050_write_reg(dev, MPU6050_REG_SMPLRT_DIV, config->sample_rate_div);
    if (ret != ESP_OK) goto err;

    // Configure gyro full-scale range
    ret = mpu6050_write_reg(dev, MPU6050_REG_GYRO_CONFIG, config->gyro_fs << 3);
    if (ret != ESP_OK) goto err;

    // Configure accel full-scale range
    ret = mpu6050_write_reg(dev, MPU6050_REG_ACCEL_CONFIG, config->accel_fs << 3);
    if (ret != ESP_OK) goto err;

    ESP_LOGI(TAG, "MPU6050 initialized: accel_fs=%d, gyro_fs=%d, dlpf=%d, div=%d",
             config->accel_fs, config->gyro_fs, config->dlpf, config->sample_rate_div);

    return ESP_OK;

err:
    free(dev);
    i2c_driver_delete(config->i2c_port);
    *handle = NULL;
    return ret;
}

esp_err_t mpu6050_deinit(mpu6050_handle_t handle)
{
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }

    // Put device to sleep
    mpu6050_write_reg(handle, MPU6050_REG_PWR_MGMT_1, 0x40);

    i2c_driver_delete(handle->i2c_port);
    free(handle);
    return ESP_OK;
}

esp_err_t mpu6050_read_raw(mpu6050_handle_t handle, mpu6050_raw_data_t *data)
{
    if (!handle || !data) {
        return ESP_ERR_INVALID_ARG;
    }

    // Read all 14 bytes (accel, temp, gyro) in one burst
    uint8_t buf[14];
    esp_err_t ret = mpu6050_read_reg(handle, MPU6050_REG_ACCEL_XOUT_H, buf, 14);
    if (ret != ESP_OK) {
        return ret;
    }

    // MPU6050 data is big-endian
    data->accel_x = (int16_t)((buf[0] << 8) | buf[1]);
    data->accel_y = (int16_t)((buf[2] << 8) | buf[3]);
    data->accel_z = (int16_t)((buf[4] << 8) | buf[5]);
    data->temp    = (int16_t)((buf[6] << 8) | buf[7]);
    data->gyro_x  = (int16_t)((buf[8] << 8) | buf[9]);
    data->gyro_y  = (int16_t)((buf[10] << 8) | buf[11]);
    data->gyro_z  = (int16_t)((buf[12] << 8) | buf[13]);

    return ESP_OK;
}

float mpu6050_get_accel_sensitivity(mpu6050_handle_t handle)
{
    // LSB/g values for each full-scale range
    static const float sens[] = {16384.0f, 8192.0f, 4096.0f, 2048.0f};
    return sens[handle->accel_fs];
}

float mpu6050_get_gyro_sensitivity(mpu6050_handle_t handle)
{
    // LSB/(deg/s) values for each full-scale range
    static const float sens[] = {131.0f, 65.5f, 32.8f, 16.4f};
    return sens[handle->gyro_fs];
}

esp_err_t mpu6050_test_connection(mpu6050_handle_t handle)
{
    uint8_t who_am_i;
    esp_err_t ret = mpu6050_read_reg(handle, MPU6050_REG_WHO_AM_I, &who_am_i, 1);
    if (ret != ESP_OK) {
        return ret;
    }

    if (who_am_i != MPU6050_WHO_AM_I_VAL) {
        ESP_LOGE(TAG, "WHO_AM_I mismatch: expected 0x%02X, got 0x%02X",
                 MPU6050_WHO_AM_I_VAL, who_am_i);
        return ESP_ERR_NOT_FOUND;
    }

    return ESP_OK;
}
