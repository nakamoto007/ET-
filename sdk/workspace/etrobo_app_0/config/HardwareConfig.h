#ifndef HARDWARE_CONFIG_H
#define HARDWARE_CONFIG_H

#include <pbio/port.h>

namespace etrobo_app {

// 左駆動モーターを接続しているSPIKE Hubのポート。
const pbio_port_id_t LEFT_MOTOR_PORT = PBIO_PORT_ID_B;
// 右駆動モーターを接続しているSPIKE Hubのポート。
const pbio_port_id_t RIGHT_MOTOR_PORT = PBIO_PORT_ID_A;
// 路面の反射値/HSV/RGBを読むカラーセンサーのポート。
const pbio_port_id_t COLOR_SENSOR_PORT = PBIO_PORT_ID_E;
// カラーセンサー昇降用モーターのポート。
const pbio_port_id_t COLOR_SENSOR_LIFT_MOTOR_PORT = PBIO_PORT_ID_C;
// スタート入力などに使うフォースセンサーのポート。
const pbio_port_id_t FORCE_SENSOR_PORT = PBIO_PORT_ID_D;
// 超音波センサーを接続するポート。使う場合だけENABLE_ULTRASONIC_SENSORをtrueにする。
const pbio_port_id_t ULTRASONIC_SENSOR_PORT = PBIO_PORT_ID_F;
// 外付けジャイロセンサーを使う場合のポート。
const pbio_port_id_t GYRO_SENSOR_PORT = PBIO_PORT_ID_F;
// 外付け加速度センサーを使う場合のポート。
const pbio_port_id_t ACCEL_SENSOR_PORT = PBIO_PORT_ID_C;

// 円周率。車輪距離や角度計算で使う。
const double PI = 3.14159265358979323846;
// 重力加速度。IMUの加速度をmm/s^2系で扱う時の基準値。
const double GRAVITY_MM_S2 = 9806.65;
// 駆動輪の直径。エンコーダ角度から走行距離を計算する。
const double WHEEL_DIAMETER_MM = 56.0;
// 左右駆動輪の間隔。旋回時の左右輪移動量計算に使う。
const double TREAD_MM = 150.0;

}  // namespace etrobo_app

#endif
