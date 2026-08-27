#ifndef ROBOT_SENSORS_H
#define ROBOT_SENSORS_H

#include <spike/pup_device.h>

namespace etrobo_app {

pup_device_t *getForceSensor(void);
bool waitForImu(void);
double calibrateMountAngle(void);
void waitForForceSensorState(bool touched_target);

}  // namespace etrobo_app

#endif
