#include "SensorLiftController.h"
#include "app.h"
#include "DriveBase.h"
#include "RobotConfig.h"

#include <pbio/error.h>
#include <spike/pup/motor.h>

namespace {

struct SensorLiftControl {
  pup_motor_t *motor;
  sensor_lift_status_t status;
  int start_count;
  int direction;
  int cycles;
  bool setup_done;
};

SensorLiftControl control = {};

int absoluteValue(int value)
{
  return value < 0 ? -value : value;
}

int signFromValue(int value)
{
  if (value < 0) {
    return -1;
  }
  return 1;
}

bool setupMotor(void)
{
  if (control.setup_done && control.motor != nullptr) {
    return true;
  }

  control.motor =
    pup_motor_get_device(etrobo_app::COLOR_SENSOR_LIFT_MOTOR_PORT);
  if (control.motor == nullptr) {
    control.status.ready = false;
    return false;
  }

  for (int retry = 0; retry < etrobo_app::MOTOR_SETUP_RETRIES; ++retry) {
    const pbio_error_t error =
      pup_motor_setup(control.motor,
                      etrobo_app::COLOR_SENSOR_LIFT_MOTOR_DIRECTION,
                      true);
    if (error == PBIO_SUCCESS) {
      control.setup_done = true;
      control.status.ready = true;
      return true;
    }
    if (error != PBIO_ERROR_AGAIN) {
      control.status.ready = false;
      return false;
    }
    dly_tsk(etrobo_app::CONTROL_PERIOD_US);
  }

  control.status.ready = false;
  return false;
}

void finishMotion(void)
{
  if (control.motor != nullptr) {
    pup_motor_brake(control.motor);
  }
  control.status.state = SENSOR_LIFT_STATE_IDLE;
  control.status.busy = false;
  control.status.command_speed = 0;
  control.direction = 0;
  control.cycles = 0;
}

}  // namespace

void sensor_lift_reset(void)
{
  control = {};
}

bool sensor_lift_start_move(int speed, int encoder_degrees)
{
  if (control.status.state == SENSOR_LIFT_STATE_MOVING) {
    return false;
  }

  if (!setupMotor()) {
    control.status.state = SENSOR_LIFT_STATE_ERROR;
    control.status.busy = false;
    return false;
  }

  const int target_degrees = absoluteValue(encoder_degrees);
  const int lift_speed = absoluteValue(etrobo_app::clampMotorSpeed(speed));
  if (target_degrees == 0 || lift_speed == 0) {
    finishMotion();
    return true;
  }

  control.start_count = pup_motor_get_count(control.motor);
  control.direction = signFromValue(encoder_degrees);
  control.cycles = 0;
  control.status.state = SENSOR_LIFT_STATE_MOVING;
  control.status.busy = true;
  control.status.target_degrees = target_degrees;
  control.status.travelled_degrees = 0;
  control.status.command_speed = control.direction * lift_speed;
  return true;
}

bool sensor_lift_start_up(void)
{
  return sensor_lift_start_move(etrobo_app::COLOR_SENSOR_LIFT_SPEED_DEG_S,
                                etrobo_app::COLOR_SENSOR_LIFT_UP_DEGREES);
}

bool sensor_lift_start_down(void)
{
  return sensor_lift_start_move(etrobo_app::COLOR_SENSOR_LIFT_SPEED_DEG_S,
                                -etrobo_app::COLOR_SENSOR_LIFT_DOWN_DEGREES);
}

void sensor_lift_step(void)
{
  if (control.status.state != SENSOR_LIFT_STATE_MOVING) {
    return;
  }
  if (control.motor == nullptr) {
    control.status.state = SENSOR_LIFT_STATE_ERROR;
    control.status.busy = false;
    return;
  }

  control.status.travelled_degrees =
    absoluteValue(static_cast<int>(pup_motor_get_count(control.motor) -
                                   control.start_count));
  if (control.status.travelled_degrees >= control.status.target_degrees) {
    finishMotion();
    return;
  }

  if (control.cycles >= etrobo_app::COLOR_SENSOR_LIFT_TIMEOUT_CYCLES) {
    pup_motor_brake(control.motor);
    control.status.state = SENSOR_LIFT_STATE_ERROR;
    control.status.busy = false;
    control.status.command_speed = 0;
    return;
  }

  pup_motor_set_speed(control.motor, control.status.command_speed);
  ++control.cycles;
}

void sensor_lift_stop(void)
{
  if (control.motor != nullptr) {
    pup_motor_brake(control.motor);
  }
  control.status.state = SENSOR_LIFT_STATE_IDLE;
  control.status.busy = false;
  control.status.command_speed = 0;
}

sensor_lift_status_t sensor_lift_get_status(void)
{
  return control.status;
}
