#ifndef DRIVE_CONTROLLER_H
#define DRIVE_CONTROLLER_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
  TURN_RESULT_OK = 0,
  TURN_RESULT_ENCODER_LIMIT = 1,
  TURN_RESULT_TIMEOUT = 2,
  TURN_RESULT_MOTOR_ERROR = -1,
} turn_result_t;

typedef enum {
  DRIVE_RESULT_OK = 0,
  DRIVE_RESULT_TIMEOUT = 2,
  DRIVE_RESULT_MOTOR_ERROR = -1,
} drive_result_t;

typedef struct {
  bool active;
  uint32_t update_count;
  int phase;
  int command_degrees;
  int direction;
  int max_speed_deg_s;
  int turn_speed_deg_s;
  int stable_count;
  int result;
  double start_heading;
  double target_degrees;
  double target_heading;
  double current_heading;
  double heading_error;
  double encoder_degrees;
  double encoder_limit_degrees;
} turn_debug_t;

typedef struct {
  bool active;
  uint32_t update_count;
  int cycle;
  int base_speed_deg_s;
  int left_speed_deg_s;
  int right_speed_deg_s;
  int result;
  double target_heading;
  double current_heading;
  double heading_error;
  double correction_deg_s;
  double correction_limit_deg_s;
  double travelled_degrees;
  double target_degrees;
} straight_debug_t;

#ifdef __cplusplus
extern "C" {
#endif

void stop_motors(void);
void reset_straight_pid_heading(void);
drive_result_t drive_straight_mm(int speed, int distance_mm);
drive_result_t drive_straight_profile_mm(int start_speed, int cruise_speed,
                                         int end_speed,
                                         int accel_distance_mm,
                                         int decel_distance_mm,
                                         int distance_mm);
drive_result_t drive_straight_mm_keep_speed(int speed, int distance_mm);
drive_result_t drive_curve_mm(int left_speed, int right_speed,
                              int distance_mm);
drive_result_t drive_curve_mm_keep_speed(int left_speed, int right_speed,
                                         int distance_mm);
drive_result_t speed_up(int start_speed, int end_speed, int distance_mm);
drive_result_t speed_down(int start_speed, int end_speed, int distance_mm);
int turn(int speed, int degrees);
turn_debug_t turn_get_debug(void);
straight_debug_t straight_get_debug(void);

#ifdef __cplusplus
}
#endif

#endif
