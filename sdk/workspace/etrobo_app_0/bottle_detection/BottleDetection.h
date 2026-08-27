#ifndef BOTTLE_DETECTION_H
#define BOTTLE_DETECTION_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
  BOTTLE_DETECTION_RESULT_OK = 0,
  BOTTLE_DETECTION_RESULT_NOT_FOUND = 1,
  BOTTLE_DETECTION_RESULT_TURN_FAILED = 2,
  BOTTLE_DETECTION_RESULT_DRIVE_TIMEOUT = 3,
  BOTTLE_DETECTION_RESULT_DEVICE_ERROR = -1,
} bottle_detection_result_t;

typedef struct {
  bool ready;
  bool object_found;
  double pose_x_mm;
  double pose_y_mm;
  double heading_deg;
  double object_x_mm;
  double object_y_mm;
  int32_t object_distance_mm;
  double object_heading_deg;
} bottle_detection_status_t;

#ifdef __cplusplus
extern "C" {
#endif

bottle_detection_result_t bottle_detection_run(void);
bottle_detection_result_t bottle_detection_navigate_to_coordinate_mm(
  double target_x_mm,
  double target_y_mm);
bottle_detection_status_t bottle_detection_get_status(void);

#ifdef __cplusplus
}
#endif

#endif
