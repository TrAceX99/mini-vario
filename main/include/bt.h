#pragma once
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Initialize NVS, NimBLE host, GAP/GATT, Nordic UART Service, and start advertising. */
int bt_init(void);

/* De-initialize (stop advertising, disconnect if needed, stop NimBLE host & disable controller). */
int bt_deinit(void);

/* Send data over Nordic UART Service TX characteristic (notifications). Returns 0 on success. */
int bt_nus_send(const void *data, uint16_t len);

/* Check if a central is connected. */
bool bt_is_connected(void);

/* Returns true if a BLE connection was ever established since boot. */
bool bt_has_ever_connected(void);

#ifdef __cplusplus
}
#endif
