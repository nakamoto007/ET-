#include "UltrasonicSensor.h"
#include "RobotConfig.h"

#include <spike/pup/ultrasonicsensor.h>

namespace {

ultrasonic_sensor_status_t status = {};
bool runtime_enabled = false;

}  // namespace

void ultrasonic_sensor_step(void)
{
  ultrasonic_sensor_status_t next = {};
  next.enabled =
    etrobo_app::ENABLE_ULTRASONIC_SENSOR && runtime_enabled;
  next.distance_mm = -1;
  if (!next.enabled) {
    status = next;
    return;
  }

  pup_device_t *sensor =
    pup_ultrasonic_sensor_get_device(etrobo_app::ULTRASONIC_SENSOR_PORT);
  next.ready = sensor != nullptr;
  if (!next.ready) {
    status = next;
    return;
  }

  next.distance_mm = pup_ultrasonic_sensor_distance(sensor);
  next.obstacle =
    next.distance_mm > 0 &&
    next.distance_mm <= etrobo_app::ULTRASONIC_OBSTACLE_DISTANCE_MM;
  status = next;
}

void ultrasonic_sensor_set_enabled(bool enabled)
{
  runtime_enabled = enabled;
  if (!runtime_enabled) {
    ultrasonic_sensor_status_t next = {};
    next.distance_mm = -1;
    status = next;
  }
}

ultrasonic_sensor_status_t ultrasonic_sensor_get_status(void)
{
  return status;
}
