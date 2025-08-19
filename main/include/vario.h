// vario.h - Variometer module API

#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
	float pressure_pa;     // Pascals
	float temperature_c;   // Celsius
	float altitude_m;      // meters
	float vspeed_mps;      // m/s
} vario_data_t;

void vario_init(void);
bool vario_get(vario_data_t *out); // returns false if not yet valid

#ifdef __cplusplus
}
#endif
