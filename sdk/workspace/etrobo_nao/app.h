#ifdef __cplusplus
extern "C" {
#endif

#include <kernel.h>
#include "BluetoothSender.h"

/* タスク優先度 */
#define MAIN_PRIORITY    5
#define ROBOT_CONTROL_PRIORITY 4
#define ROBOT_SENSOR_PRIORITY 6
#define BLUETOOTH_CONNECTION_PRIORITY 7
#define SENSOR_LOG_PRIORITY 8

/* 周期タスク。単位はdly_tsk()と同じマイクロ秒。 */
#define ROBOT_CONTROL_PERIOD (3 * 1000)
#define ROBOT_SENSOR_PERIOD  (5 * 1000)

extern void sensor_log_task(intptr_t unused);
extern void robot_control_task(intptr_t unused);
extern void robot_sensor_task(intptr_t unused);

#ifndef STACK_SIZE
#define STACK_SIZE      (4096)
#endif

#ifndef TOPPERS_MACRO_ONLY

extern void main_task(intptr_t exinf);

#endif

#ifdef __cplusplus
}
#endif
