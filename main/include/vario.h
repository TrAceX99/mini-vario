// vario.h - Variometer module API

#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
	double pressure_pa;
	double temperature_c;
	float altitude_m;
	float vspeed_mps;
} vario_data_t;

void vario_init(void);
bool vario_get(vario_data_t *out); // returns false if not yet valid

#ifdef __cplusplus
}
#endif
