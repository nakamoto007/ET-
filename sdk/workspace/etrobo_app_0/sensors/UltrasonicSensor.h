#ifndef ETROBO_APP_ULTRASONIC_SENSOR_H
#define ETROBO_APP_ULTRASONIC_SENSOR_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
  bool enabled;
  bool ready;
  bool obstacle;
  int32_t distance_mm;
} ultrasonic_sensor_status_t;

#ifdef __cplusplus
extern "C" {
#endif

void ultrasonic_sensor_step(void);
void ultrasonic_sensor_set_enabled(bool enabled);
ultrasonic_sensor_status_t ultrasonic_sensor_get_status(void);

#ifdef __cplusplus
}
#endif

#endif
