#ifndef BOTTLE_CONFIG_H
#define BOTTLE_CONFIG_H

#include "ControlConfig.h"

namespace etrobo_app {

// 超音波センサーを読むか。未接続の時はfalseにしておく。
const bool ENABLE_ULTRASONIC_SENSOR = true;
// 超音波センサーで障害物停止を使うか。
const bool ENABLE_ULTRASONIC_STOP = true;
// ボトル検知へ向かうライントレースで、開始から超音波センサーを有効化するまでの待ち時間。
const int LINE_TRACE_TO_BOTTLE_ULTRASONIC_START_DELAY_US = 5 * 1000 * 1000;
// ボトル検知ライントレース開始前、センサーアームを下げながら直進する速度 [deg/s]。
const int LINE_TRACE_TO_BOTTLE_ARM_LOWER_SPEED_DEG_S = 120;
// センサーアーム下降中に1回のPID直進で進む距離 [mm]。小さいほど完了時の余走が少ない。
const int LINE_TRACE_TO_BOTTLE_ARM_LOWER_STEP_MM = 5;
// センサーアーム下降待ちで直進してよい最大距離 [mm]。超えたら異常として止める。
const int LINE_TRACE_TO_BOTTLE_ARM_LOWER_MAX_DISTANCE_MM = 80;
// 障害物ありとみなす距離 [mm]。
const int ULTRASONIC_OBSTACLE_DISTANCE_MM = 150;
// 超音波検知後、色検知位置へ近づく低速PID直進の速度 [deg/s]。
const int BOTTLE_COLOR_APPROACH_SPEED_DEG_S = 120;
// 超音波検知後、色検知位置へ近づく低速PID直進の距離 [mm]。
const int BOTTLE_COLOR_APPROACH_DISTANCE_MM = 60;
// ボトル色判定で読むサンプル数。
const int BOTTLE_COLOR_SAMPLE_COUNT = 5;
// ボトル色判定のサンプル間隔。30ms。
const int BOTTLE_COLOR_SAMPLE_INTERVAL_US = 30 * 1000;
// 色別搬送中、青ゾーン/黒検知を見ながらライントレースする最大周期。
const int BOTTLE_CARRY_COLOR_LINE_TRACE_TIMEOUT_CYCLES = 6000;
// 色別搬送中の色判定ポーリング周期。
const int BOTTLE_CARRY_COLOR_POLL_INTERVAL_US = 10 * 1000;
// 色別搬送中、同じ色のゾーンを探すPID直進速度 [deg/s]。
const int BOTTLE_COLOR_ZONE_SEARCH_SPEED_DEG_S = 120;
// 色別搬送中、同じ色のゾーンを探す1回あたりのPID直進距離 [mm]。
const int BOTTLE_COLOR_ZONE_SEARCH_STEP_MM = 10;
// 色別搬送中、同じ色のゾーンを探す最大距離 [mm]。
const int BOTTLE_COLOR_ZONE_SEARCH_MAX_DISTANCE_MM = 1000;
// 色別搬送中、同じ色のゾーン到達後に戻るPID後退速度 [deg/s]。
const int BOTTLE_COLOR_ZONE_RETURN_SPEED_DEG_S = 120;
// 一番下の青を通過して黒を探す最大距離 [mm]。
const int BOTTLE_BOTTOM_BLUE_TO_BLACK_MAX_DISTANCE_MM = 1000;

// 難所後、押し出し位置へ移動する距離 [mm]。未調整なので初期値は0。
const int BOTTLE_PUSH_APPROACH_DISTANCE_MM = 0;
// 押し出し区間のPID直進速度 [deg/s]。
const int BOTTLE_PUSH_SPEED_DEG_S = 120;
// 押し出し区間で1回に進む距離 [mm]。
const int BOTTLE_PUSH_STEP_MM = 10;
// 押し出し区間で進んでよい最大距離 [mm]。
const int BOTTLE_PUSH_MAX_DISTANCE_MM = 250;
// ゴールへ向かう前に旋回する角度 [deg]。未調整なので初期値は0。
const int GOAL_RUN_TURN_DEGREES = 0;
// ゴールへ向かうPID直進距離 [mm]。未調整なので初期値は0。
const int GOAL_RUN_DISTANCE_MM = 0;
// ゴールへ向かうPID直進速度 [deg/s]。
const int GOAL_RUN_SPEED_DEG_S = 200;

// ボトル探索で左端として向く角度。走行開始時の正面を0度とする。
const int BOTTLE_DETECTION_SCAN_START_DEG = -90;
// ボトル探索で右端として向く角度。
const int BOTTLE_DETECTION_SCAN_END_DEG = 90;
// ボトル探索中に何度刻みで距離を読むか。
const int BOTTLE_DETECTION_SCAN_STEP_DEG = 1;
// ボトル検出として採用する最大距離 [mm]。
const int BOTTLE_DETECTION_MAX_DISTANCE_MM = 500;
// 検出したボトル座標より何mm先まで進ませるか。
const int BOTTLE_DETECTION_COLLISION_MARGIN_MM = 150;
// ボトル座標へ接近するときの到達許容距離 [mm]。
const double BOTTLE_DETECTION_GOAL_TOLERANCE_MM = 30.0;
// 探索開始角やボトル方向へ向くときの旋回速度上限 [deg/s]。
const int BOTTLE_DETECTION_TURN_SPEED_DEG_S = 160;
// 探索中の大きな角度差で使う旋回速度 [deg/s]。
const int BOTTLE_DETECTION_SCAN_MAX_SPEED_DEG_S = 120;
// 探索中の中くらいの角度差で使う旋回速度 [deg/s]。
const int BOTTLE_DETECTION_SCAN_MIDDLE_SPEED_DEG_S = 80;
// 探索中の目標角付近で使う最低旋回速度 [deg/s]。
const int BOTTLE_DETECTION_SCAN_MIN_SPEED_DEG_S = 50;
// 探索/向き合わせで目標角に到達したとみなす許容誤差。
const double BOTTLE_DETECTION_TURN_TOLERANCE_DEG = 0.5;
// ボトルへ進む時の基本速度 [deg/s]。
const int BOTTLE_DETECTION_DRIVE_SPEED_DEG_S = 250;
// ボトルへ進む時に、方位誤差を左右速度差へ変換する比例ゲイン。
const double BOTTLE_DETECTION_NAV_KP = STRAIGHT_PID_KP;
// ボトルへ進む時の積分ゲイン。通常は0のまま使う。
const double BOTTLE_DETECTION_NAV_KI = 0.0;
// ボトルへ進む時の微分ゲイン。
const double BOTTLE_DETECTION_NAV_KD = STRAIGHT_PID_KD;
// ボトル接近PIDの積分項上限。
const double BOTTLE_DETECTION_NAV_INTEGRAL_LIMIT_DEG_SEC =
  STRAIGHT_PID_INTEGRAL_LIMIT_DEG_SEC;
// ボトル接近PIDで左右速度へ足し引きする補正量の上限 [deg/s]。
const double BOTTLE_DETECTION_NAV_CORRECTION_LIMIT_DEG_S = 160.0;
// 探索中、停止後に超音波センサーの値が落ち着くまで待つ時間。
const int BOTTLE_DETECTION_SENSOR_SETTLE_TIME_US = 40 * 1000;
// 超音波センサーを連続で読む間隔。
const int BOTTLE_DETECTION_SAMPLE_INTERVAL_US = 10 * 1000;
// 中央値を取るために使う超音波サンプル数。奇数にする。
const int BOTTLE_DETECTION_DISTANCE_SAMPLE_COUNT = 5;
// 有効な超音波値が揃うまでの最大試行回数。
const int BOTTLE_DETECTION_DISTANCE_MAX_ATTEMPTS = 25;
// 探索中、1つの目標角へ向くために使える最大制御周期数。
const int BOTTLE_DETECTION_SCAN_STEP_TIMEOUT_CYCLES = 150;
// ボトルへ進む処理の安全停止用最大周期数。2000 * 10ms = 約20秒。
const int BOTTLE_DETECTION_NAVIGATION_TIMEOUT_CYCLES = 2000;

}  // namespace etrobo_app

#endif
