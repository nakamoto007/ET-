#include "app.h"
#include "RobotSensors.h"
#include "RobotConfig.h"

#include <cmath>

#include <spike/hub/imu.h>
#include <spike/pup/forcesensor.h>

namespace etrobo_app {

pup_device_t *getForceSensor(void)
{
  return pup_force_sensor_get_device(FORCE_SENSOR_PORT);
}

bool waitForImu(void)
{
  for (int retry = 0; retry < IMU_READY_RETRIES; ++retry) {
    if (hub_imu_is_ready()) {
      return true;
    }
    dly_tsk(100 * 1000);
  }
  return false;
}

double calibrateMountAngle(void)
{
  double accel_x_sum = 0.0;
  double accel_z_sum = 0.0;

  for (int sample = 0; sample < CALIBRATION_SAMPLES; ++sample) {
    float accel[3];
    hub_imu_get_acceleration(accel);
    accel_x_sum += accel[0];
    accel_z_sum += accel[2];
    dly_tsk(CONTROL_PERIOD_US);
  }

  const double accel_x_average = accel_x_sum / CALIBRATION_SAMPLES;
  const double accel_z_average = accel_z_sum / CALIBRATION_SAMPLES;
  return std::atan2(-accel_x_average, accel_z_average) * 180.0 / PI;
}

void waitForForceSensorState(bool touched_target)
{
  pup_device_t *force = getForceSensor();
  if (force == nullptr) {
    return;
  }

  while (pup_force_sensor_touched(force) != touched_target) {
    dly_tsk(CONTROL_PERIOD_US);
  }
}

}  // namespace etrobo_app
