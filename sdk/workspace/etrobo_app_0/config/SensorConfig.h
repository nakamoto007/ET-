#ifndef SENSOR_CONFIG_H
#define SENSOR_CONFIG_H

#include <spike/pup/motor.h>

namespace etrobo_app {

// カラーセンサー反射値の取りうる範囲。
const int COLOR_SENSOR_REFLECTION_MIN = 0;
const int COLOR_SENSOR_REFLECTION_MAX = 100;
// 白黒キャリブレーション前に使う正規化の初期値。
const int COLOR_SENSOR_DEFAULT_NORMALIZE_BLACK_REFLECTION = 10;
const int COLOR_SENSOR_DEFAULT_NORMALIZE_WHITE_REFLECTION = 80;
// 白黒反射値キャリブレーションで平均するサンプル数。
const int COLOR_REFLECTION_CALIBRATION_SAMPLE_COUNT = 10;
// 白黒反射値キャリブレーションのサンプル間隔。
const int COLOR_REFLECTION_CALIBRATION_SAMPLE_INTERVAL_US = 20 * 1000;
// 正規化後の反射値スケール。黒=0、白=100、ライン境界目標=50。
const int COLOR_SENSOR_NORMALIZED_REFLECTION_MIN = 0;
const int COLOR_SENSOR_NORMALIZED_REFLECTION_MAX = 100;
const int COLOR_SENSOR_NORMALIZED_TARGET_REFLECTION = 60;
// カラーセンサーRGB生値の取りうる範囲。SPIKEのカラーセンサーは10bit相当。
const int COLOR_SENSOR_RGB_RAW_MIN = 0;
const int COLOR_SENSOR_RGB_RAW_MAX = 1024;
// HSVの色相Hの取りうる範囲。単位は度。
const int COLOR_SENSOR_HUE_MIN_DEGREES = 0;
const int COLOR_SENSOR_HUE_MAX_DEGREES = 359;
// HSVの彩度S/明度Vの取りうる範囲。0-100の百分率。
const int COLOR_SENSOR_HSV_PERCENT_MIN = 0;
const int COLOR_SENSOR_HSV_PERCENT_MAX = 100;
// HSVの明度Vを8bit換算した値の取りうる範囲。
const int COLOR_SENSOR_HSV_VALUE8_MIN = 0;
const int COLOR_SENSOR_HSV_VALUE8_MAX = 255;

// カラーセンサー昇降モーターの正転方向。
const pup_direction_t COLOR_SENSOR_LIFT_MOTOR_DIRECTION =
  PUP_DIRECTION_COUNTERCLOCKWISE;
// カラーセンサー昇降モーターを動かす時の速度 [deg/s]。
const int COLOR_SENSOR_LIFT_SPEED_DEG_S = 900;
// カラーセンサーを上げる時に回すエンコーダ角度。
const int COLOR_SENSOR_LIFT_UP_DEGREES = 220;
// カラーセンサーを下げる時に回すエンコーダ角度。
const int COLOR_SENSOR_LIFT_DOWN_DEGREES = 220;
// カラーセンサー昇降処理の安全停止用最大周期数。100回 * 5ms = 約0.5秒。
const int COLOR_SENSOR_LIFT_TIMEOUT_CYCLES = 100;

}  // namespace etrobo_app

#endif
