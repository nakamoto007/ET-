#ifndef ALL_SENSORS_H
#define ALL_SENSORS_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
  bool drive_motors_ready;
  int32_t left_motor_count;
  int32_t right_motor_count;
  int32_t left_motor_speed;
  int32_t right_motor_speed;
  int32_t left_motor_power;
  int32_t right_motor_power;
} drive_motor_values_t;

typedef struct {
  bool color_sensor_ready;
  int32_t reflection;
  int32_t normalized_reflection;
  int32_t ambient;
  uint16_t rgb_r;
  uint16_t rgb_g;
  uint16_t rgb_b;
  uint16_t hsv_h;
  uint8_t hsv_s;
  uint8_t hsv_v;
  uint8_t hsv_v8;
} color_sensor_values_t;

typedef struct {
  bool force_sensor_ready;
  bool touched;
  float force_n;
  float distance_mm;
} force_sensor_values_t;

typedef struct {
  bool imu_ready;
  bool stationary;
  int calibration_status;
  float acceleration_x;
  float acceleration_y;
  float acceleration_z;
  float angular_velocity_x;
  float angular_velocity_y;
  float angular_velocity_z;
  // SPIKE側の3D heading。アプリ独自の時間比例ドリフト補正は未適用。
  float raw_heading;
  // 走行制御が使用するheading。独自補正無効時はraw_headingと同じ系統。
  float heading;
  // アプリ独自補正で差し引く推定ドリフト量 [deg/min]。
  float heading_drift_rate;
  float temperature;
} imu_values_t;

typedef struct {
  drive_motor_values_t drive_motors;
  color_sensor_values_t color;
  force_sensor_values_t force;
  imu_values_t imu;
} all_sensor_values_t;

#ifdef __cplusplus
extern "C" {
#endif

all_sensor_values_t get_all_sensor_values(void);
bool get_all_sensor_values_into(all_sensor_values_t *values);

#ifdef __cplusplus
}
#endif

#endif
