// battery.h - Battery measurement API
#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Initialize battery measurement (starts background task)
void battery_init(void);

// Get filtered battery percentage (0.0 - 1.0). Returns -1 if not yet valid.
float battery_get(void);

// Optional: get filtered battery voltage in volts.
float battery_get_voltage(void);

#ifdef __cplusplus
}
#endif
