#ifndef CHALLENGE_CONFIG_H
#define CHALLENGE_CONFIG_H

namespace etrobo_app {

// trueにすると起動後のメイン処理を難所だけのテストにする。
// 通常の競技フローへ戻す時はfalseにする。
const bool RUN_CHALLENGE_ONLY_TEST = true;

// 難所攻略のステップ文字列で、F 1文字が表す前進距離 [mm]。
const int CHALLENGE_STEP_FORWARD_DISTANCE_MM = 125;
// 難所攻略のステップ文字列で、F を走る時の速度 [deg/s]。
const int CHALLENGE_STEP_FORWARD_SPEED_DEG_S = 800;
// 難所ステップ走行の開始速度 [deg/s]。
const int CHALLENGE_STEP_START_SPEED_DEG_S = 300;
// 難所ステップ走行の加速に使う距離 [mm]。
const int CHALLENGE_STEP_ACCEL_DISTANCE_MM = 75;
// 難所ステップ走行の終端で減速に使う距離 [mm]。
const int CHALLENGE_STEP_DECEL_DISTANCE_MM = 75;
// 難所攻略のステップ文字列で、L/R の90度旋回に使う速度 [deg/s]。
const int CHALLENGE_STEP_TURN_SPEED_DEG_S = 160;

}  // namespace etrobo_app

#endif
