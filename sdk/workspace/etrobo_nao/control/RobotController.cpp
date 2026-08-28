#include "app.h"
#include "RobotController.h"
#include "DriveBase.h"
#include "DriveController.h"
#include "RobotSensors.h"

#include <pbio/error.h>
#include <spike/hub/imu.h>
#include <spike/pup/motor.h>

robot_init_result_t initialize_robot(void)
{
  etrobo_app::DriveMotors motors;
  pup_device_t *force = etrobo_app::getForceSensor();
  if (!etrobo_app::getDriveMotors(&motors) || force == nullptr) {
    return ROBOT_INIT_DEVICE_ERROR;
  }

  if (!etrobo_app::setupDriveMotor(motors.left,
                                   PUP_DIRECTION_COUNTERCLOCKWISE) ||
      !etrobo_app::setupDriveMotor(motors.right, PUP_DIRECTION_CLOCKWISE)) {
    return ROBOT_INIT_MOTOR_ERROR;
  }

  etrobo_app::stopDriveMotors(motors);

  const pbio_error_t imu_error = hub_imu_init();
  if (imu_error != PBIO_SUCCESS) {
    return ROBOT_INIT_IMU_ERROR;
  }
  if (!etrobo_app::waitForImu()) {
    return ROBOT_INIT_IMU_TIMEOUT;
  }

  etrobo_app::brakeMotors(motors);
  return ROBOT_INIT_OK;
}

void wait_for_force_start(void)
{
  etrobo_app::waitForForceSensorState(true);
  etrobo_app::waitForForceSensorState(false);
}

void calibrate_robot_pose(void)
{
  etrobo_app::DriveMotors motors;
  if (!etrobo_app::getDriveMotors(&motors)) {
    return;
  }

  dly_tsk(300 * 1000);

  const double mount_angle_deg = etrobo_app::calibrateMountAngle();
  hub_imu_set_tilt(static_cast<float>(mount_angle_deg));
  hub_imu_reset_heading();
  dly_tsk(5 * 1000 * 1000);
  hub_imu_reset_heading();
  reset_straight_pid_heading();
  etrobo_app::resetDriveMotorCounts(motors);
}
