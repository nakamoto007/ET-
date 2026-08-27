#ifndef ROBOT_STATE_CONTROLLER_H
#define ROBOT_STATE_CONTROLLER_H

#include "LineTraceController.h"

#include <stdbool.h>

typedef enum {
  ROBOT_RUN_STATE_IDLE = 0,
  ROBOT_RUN_STATE_LINE_TRACE,
  ROBOT_RUN_STATE_STOPPED,
  ROBOT_RUN_STATE_ERROR,
} robot_run_state_t;

typedef struct {
  robot_run_state_t state;
  bool running;
  line_trace_result_t line_trace_result;
} robot_state_status_t;

#ifdef __cplusplus
extern "C" {
#endif

enum {
  ROBOT_LINE_TRACE_DISTANCE_UNLIMITED_MM = -1,
  LINE_TRACE_ULTRASONIC_DISABLED_US = -1,
};

typedef struct {
  int distance_mm;
  int ultrasonic_start_delay_us;
  bool update_color_during_line_trace;
} robot_line_trace_options_t;

void robot_state_controller_reset(void);
line_trace_result_t robot_state_controller_run_line_trace(
  const robot_line_trace_options_t *options);
bool robot_state_controller_start_line_trace(
  const robot_line_trace_options_t *options);
void robot_state_controller_step(void);
void robot_sensor_services_step(void);
void robot_state_controller_stop(void);
bool robot_state_controller_is_running(void);
robot_state_status_t robot_state_controller_get_status(void);

#ifdef __cplusplus
}
#endif

#endif
