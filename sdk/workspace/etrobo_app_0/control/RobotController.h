#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

#include "DriveController.h"

typedef enum {
  ROBOT_INIT_OK = 0,
  ROBOT_INIT_DEVICE_ERROR,
  ROBOT_INIT_MOTOR_ERROR,
  ROBOT_INIT_IMU_ERROR,
  ROBOT_INIT_IMU_TIMEOUT,
} robot_init_result_t;

#ifdef __cplusplus
extern "C" {
#endif

robot_init_result_t initialize_robot(void);
void wait_for_force_start(void);
void calibrate_robot_pose(void);

#ifdef __cplusplus
}
#endif

#endif
