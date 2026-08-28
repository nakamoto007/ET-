#include "app.h"
#include "DriveBase.h"

#include <cmath>

#include <pbio/error.h>
#include <spike/hub/imu.h>
#include <spike/pup/motor.h>

namespace {

double clampDouble(double value, double limit)
{
  const double absolute_limit = std::fabs(limit);
  if (value > absolute_limit) {
    return absolute_limit;
  }
  if (value < -absolute_limit) {
    return -absolute_limit;
  }
  return value;
}

int normalizeDriveDirection(int direction)
{
  return direction < 0 ? -1 : 1;
}

}  // namespace

namespace etrobo_app {

int clampMotorSpeed(double speed_deg_s)
{
  if (speed_deg_s > MOTOR_SPEED_LIMIT_DEG_S) {
    return MOTOR_SPEED_LIMIT_DEG_S;
  }
  if (speed_deg_s < -MOTOR_SPEED_LIMIT_DEG_S) {
    return -MOTOR_SPEED_LIMIT_DEG_S;
  }
  return static_cast<int>(speed_deg_s);
}

bool getDriveMotors(DriveMotors *motors)
{
  motors->left = pup_motor_get_device(LEFT_MOTOR_PORT);
  motors->right = pup_motor_get_device(RIGHT_MOTOR_PORT);
  return motors->left != nullptr && motors->right != nullptr;
}

bool setupDriveMotor(pup_motor_t *motor, pup_direction_t direction)
{
  for (int retry = 0; retry < MOTOR_SETUP_RETRIES; ++retry) {
    const pbio_error_t error = pup_motor_setup(motor, direction, true);
    if (error == PBIO_SUCCESS) {
      return true;
    }
    if (error != PBIO_ERROR_AGAIN) {
      return false;
    }
    dly_tsk(CONTROL_PERIOD_US);
  }
  return false;
}

double encoderDegreesToMm(int32_t degrees)
{
  return static_cast<double>(degrees) * PI * WHEEL_DIAMETER_MM / 360.0;
}

double mmToEncoderDegrees(double mm)
{
  return mm * 360.0 / (PI * WHEEL_DIAMETER_MM);
}

double averageWheelDistanceMm(double left_mm, double right_mm)
{
  return (left_mm + right_mm) / 2.0;
}

void resetDriveMotorCounts(const DriveMotors &motors)
{
  pup_motor_reset_count(motors.left);
  pup_motor_reset_count(motors.right);
}

void setStraightSpeed(const DriveMotors &motors, int speed_deg_s)
{
  const int speed = clampMotorSpeed(speed_deg_s);
  pup_motor_set_speed(motors.left, speed);
  pup_motor_set_speed(motors.right, speed);
}

void setMotorSpeeds(const DriveMotors &motors,
                    double left_speed_deg_s, double right_speed_deg_s)
{
  pup_motor_set_speed(motors.left, clampMotorSpeed(left_speed_deg_s));
  pup_motor_set_speed(motors.right, clampMotorSpeed(right_speed_deg_s));
}

StraightCorrectionState beginStraightCorrection(const DriveMotors &motors)
{
  StraightCorrectionState state;
  state.left_start_count = pup_motor_get_count(motors.left);
  state.right_start_count = pup_motor_get_count(motors.right);
  state.start_heading = hub_imu_get_heading();
  return state;
}

double calculateStraightCorrection(const DriveMotors &motors,
                                   const StraightCorrectionState &state,
                                   int drive_direction)
{
  const int direction = normalizeDriveDirection(drive_direction);
  const int32_t left_delta =
    pup_motor_get_count(motors.left) - state.left_start_count;
  const int32_t right_delta =
    pup_motor_get_count(motors.right) - state.right_start_count;
  const double left_mm = encoderDegreesToMm(left_delta);
  const double right_mm = encoderDegreesToMm(right_delta);
  const double encoder_yaw_degrees =
    direction * (right_mm - left_mm) / TREAD_MM * 180.0 / PI;
  const double gyro_yaw_degrees =
    hub_imu_get_heading() - state.start_heading;
  const double correction =
    -(gyro_yaw_degrees * STRAIGHT_GYRO_CORRECTION_GAIN +
      encoder_yaw_degrees * STRAIGHT_ENCODER_CORRECTION_GAIN);

  return clampDouble(correction, STRAIGHT_CORRECTION_SPEED_LIMIT_DEG_S);
}

void setCorrectedStraightSpeed(const DriveMotors &motors,
                               int base_speed_deg_s,
                               const StraightCorrectionState &state)
{
  if (base_speed_deg_s == 0) {
    setStraightSpeed(motors, 0);
    return;
  }

  const int direction = base_speed_deg_s < 0 ? -1 : 1;
  const double correction =
    calculateStraightCorrection(motors, state, direction);
  const double speed_correction = direction * correction;
  setMotorSpeeds(motors,
                 base_speed_deg_s + speed_correction,
                 base_speed_deg_s - speed_correction);
}

void brakeMotors(const DriveMotors &motors)
{
  pup_motor_brake(motors.left);
  pup_motor_brake(motors.right);
}

void stopDriveMotors(const DriveMotors &motors)
{
  pup_motor_stop(motors.left);
  pup_motor_stop(motors.right);
}

}  // namespace etrobo_app
