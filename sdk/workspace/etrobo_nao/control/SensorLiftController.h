#ifndef SENSOR_LIFT_CONTROLLER_H
#define SENSOR_LIFT_CONTROLLER_H

#include <stdbool.h>

typedef enum {
  SENSOR_LIFT_STATE_IDLE = 0,
  SENSOR_LIFT_STATE_MOVING,
  SENSOR_LIFT_STATE_ERROR,
} sensor_lift_state_t;

typedef struct {
  sensor_lift_state_t state;
  bool busy;
  bool ready;
  int target_degrees;
  int travelled_degrees;
  int command_speed;
} sensor_lift_status_t;

#ifdef __cplusplus
extern "C" {
#endif

void sensor_lift_reset(void);
bool sensor_lift_start_move(int speed, int encoder_degrees);
bool sensor_lift_start_up(void);
bool sensor_lift_start_down(void);
void sensor_lift_step(void);
void sensor_lift_stop(void);
sensor_lift_status_t sensor_lift_get_status(void);

#ifdef __cplusplus
}
#endif

#endif
