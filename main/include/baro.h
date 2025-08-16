// baro.h - Public interface for barometer (BMP3) driver wrapper
//
// Provides initialization and read functions for the barometer sensor.
// Implementation located in baro.c.

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// Initialize I2C device and configure BMP3 sensor.
// This must be called once before any calls to baro_read().
void baro_init(void);

// Read current pressure (Pa) and temperature (°C) from the sensor.
// Parameters must be valid non-null pointers.
// On failure, values are set to 0.
void baro_read(double *pressure, double *temperature);

#ifdef __cplusplus
}
#endif

