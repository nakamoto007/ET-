#ifndef LINE_TRACER_H
#define LINE_TRACER_H

#include "LineTraceConfig.h"

#include <pbio/port.h>
#include <stdbool.h>
#include <stdint.h>

typedef struct {
  int reflection;
  int normalized_reflection;
  int error;
  int derivative;
  int base_power;
  int line_turn_power;
  int imu_turn_power;
  int turn_power;
  int left_power;
  int right_power;
  int edge;
  int path_state;
  int curve_level;
  int curve_entry;
} line_tracer_debug_t;

  extern void LineTracer_Configure(pbio_port_id_t left_motor_port, pbio_port_id_t right_motor_port, pbio_port_id_t color_sensor_port);
  extern bool LineTracer_Run(void);
  extern line_tracer_debug_t LineTracer_GetDebug(void);

#endif
