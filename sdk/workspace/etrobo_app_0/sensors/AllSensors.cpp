#include "AllSensors.h"
#include "ColorSensorService.h"
#include "DriveBase.h"
#include "HubIMUCorrection.h"
#include "RobotConfig.h"
#include "RobotSensors.h"

#include <spike/hub/imu.h>
#include <spike/pup/forcesensor.h>
#include <spike/pup/motor.h>

namespace {

void readDriveMotorValues(drive_motor_values_t *values)
{
  etrobo_app::DriveMotors motors;
  values->drive_motors_ready = etrobo_app::getDriveMotors(&motors);
  if (!values->drive_motors_ready) {
    return;
  }

  values->left_motor_count = pup_motor_get_count(motors.left);
  values->right_motor_count = pup_motor_get_count(motors.right);
  values->left_motor_speed = pup_motor_get_speed(motors.left);
  values->right_motor_speed = pup_motor_get_speed(motors.right);
  values->left_motor_power = pup_motor_get_power(motors.left);
  values->right_motor_power = pup_motor_get_power(motors.right);
}

void readColorSensorValues(color_sensor_values_t *values)
{
  (void)color_sensor_service_get_values(values);
}

void readForceSensorValues(force_sensor_values_t *values)
{
  pup_device_t *force_sensor = etrobo_app::getForceSensor();
  values->force_sensor_ready = force_sensor != nullptr;
  if (!values->force_sensor_ready) {
    return;
  }

  values->touched = pup_force_sensor_touched(force_sensor);
  values->force_n = pup_force_sensor_force(force_sensor);
  values->distance_mm = pup_force_sensor_distance(force_sensor);
}

void readImuValues(imu_values_t *values)
{
  values->calibration_status = hub_imu_calibration_status;
  values->imu_ready = hub_imu_is_ready();
  if (!values->imu_ready) {
    return;
  }

  float acceleration[3];
  float angular_velocity[3];
  hub_imu_get_acceleration(acceleration);
  hub_imu_get_angular_velocity(angular_velocity);

  values->stationary = hub_imu_is_stationary();
  values->acceleration_x = acceleration[0];
  values->acceleration_y = acceleration[1];
  values->acceleration_z = acceleration[2];
  values->angular_velocity_x = angular_velocity[0];
  values->angular_velocity_y = angular_velocity[1];
  values->angular_velocity_z = angular_velocity[2];
  values->raw_heading = hub_imu_get_raw_heading();
  values->heading = hub_imu_get_heading();
  values->heading_drift_rate = hub_imu_get_heading_drift_rate();
  values->temperature = hub_imu_get_temperature();
}

}  // namespace

all_sensor_values_t get_all_sensor_values(void)
{
  all_sensor_values_t values = {};
  get_all_sensor_values_into(&values);
  return values;
}

bool get_all_sensor_values_into(all_sensor_values_t *values)
{
  if (values == nullptr) {
    return false;
  }

  *values = {};
  readDriveMotorValues(&values->drive_motors);
  readColorSensorValues(&values->color);
  readForceSensorValues(&values->force);
  readImuValues(&values->imu);

  return values->drive_motors.drive_motors_ready &&
         values->color.color_sensor_ready &&
         values->force.force_sensor_ready &&
         values->imu.imu_ready;
}
