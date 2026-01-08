#pragma once

#include <stdbool.h>
#include <stddef.h>

/**
 * @brief Initialize BLE and start advertising
 */
void ble_service_init(void);

/**
 * @brief Check if a client is connected
 */
bool ble_is_connected(void);

/**
 * @brief Send stream packet notification
 * @param data Packet data
 * @param len Packet length
 * @return true if sent successfully
 */
bool ble_notify_stream(const void *data, size_t len);

/**
 * @brief Update status characteristic
 * @param data Status data
 * @param len Data length
 */
void ble_update_status(const void *data, size_t len);
