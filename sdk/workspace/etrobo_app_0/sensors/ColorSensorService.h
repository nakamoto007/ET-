#ifndef COLOR_SENSOR_SERVICE_H
#define COLOR_SENSOR_SERVICE_H

#include "AllSensors.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

bool color_sensor_service_update(void);
bool color_sensor_service_update_reflection(void);
void color_sensor_service_set_reflection_only(bool enabled);
bool color_sensor_service_is_reflection_only(void);
bool color_sensor_service_lock(void);
void color_sensor_service_unlock(void);
bool color_sensor_service_set_normalization_reflection(int32_t black_reflection,
                                                       int32_t white_reflection);
int32_t color_sensor_service_normalize_reflection(int32_t reflection);
void color_sensor_service_store_reflection(int32_t reflection);
bool color_sensor_service_get_values(color_sensor_values_t *values);

#ifdef __cplusplus
}
#endif

#endif
