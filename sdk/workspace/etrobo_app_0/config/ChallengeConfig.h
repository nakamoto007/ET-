#ifndef CHALLENGE_CONFIG_H
#define CHALLENGE_CONFIG_H

namespace etrobo_app {

// S字走行の直進区間で使う速度 [deg/s]。
const int S_CURVE_STRAIGHT_SPEED_DEG_S = 350;
// S字走行の外輪側速度 [deg/s]。内輪との差が大きいほど急カーブになる。
const int S_CURVE_FAST_SPEED_DEG_S = 450;
// S字走行の内輪側速度 [deg/s]。
const int S_CURVE_SLOW_SPEED_DEG_S = 300;
// S字走行前に少し姿勢を安定させる直進距離。
const int S_CURVE_ENTRY_DISTANCE_MM = 120;
// 片側カーブの走行距離。長くするとS字が大きくなる。
const int S_CURVE_ARC_DISTANCE_MM = 400;
// S字走行後に減速しながら進む距離。
const int S_CURVE_EXIT_DISTANCE_MM = 150;
// trueにするとスタート後にS字走行を実行する。
const bool RUN_S_CURVE = false;
// 8の字走行の外輪側速度 [deg/s]。
const int FIGURE_EIGHT_FAST_SPEED_DEG_S = 420;
// 8の字走行の内輪側速度 [deg/s]。
const int FIGURE_EIGHT_SLOW_SPEED_DEG_S = 220;
// 8の字走行前に少し直進する距離。
const int FIGURE_EIGHT_ENTRY_DISTANCE_MM = 100;
// 8の字の片側ループ距離。長くすると大きな輪になる。
const int FIGURE_EIGHT_LOOP_DISTANCE_MM = 1500;
// 8の字走行後に減速しながら進む距離。
const int FIGURE_EIGHT_EXIT_DISTANCE_MM = 150;
// trueにするとスタート後に8の字走行を実行する。
const bool RUN_FIGURE_EIGHT = true;
// trueにすると起動後のメイン処理を難所だけのテストにする。
// 通常の競技フローへ戻す時はfalseにする。
const bool RUN_CHALLENGE_ONLY_TEST = true;

// 難所攻略のFコマンドで、count=1が表す前進距離 [mm]。
const int CHALLENGE_STEP_FORWARD_DISTANCE_MM = 130;
// 難所攻略のFコマンドを走る時の速度 [deg/s]。
const int CHALLENGE_STEP_FORWARD_SPEED_DEG_S = 500;
// 難所攻略のBコマンドで、count=1が表す後退距離 [mm]。
// 前進とはタイヤの滑りや機体荷重が異なるため、実測距離に合わせて独立調整する。
// 現在は130mm。B単独の実測距離がずれる場合だけ、この値を調整する。
const int CHALLENGE_STEP_BACKWARD_DISTANCE_MM = 130;
// 難所攻略のBコマンドを走る時の速度 [deg/s]。
// 後退時の滑りが大きい場合、前進速度を変えずにこの値だけ下げて比較できる。
const int CHALLENGE_STEP_BACKWARD_SPEED_DEG_S = 500;
// Bの直後にFが続く時だけ、固定距離補償、位置保持、前進の緩加速を行う。
const bool ENABLE_CHALLENGE_BACKWARD_TO_FORWARD_CONTROL = true;
// 後退完了位置をholdする時間。旋回を挟むB→R→Fには適用しない。
const int CHALLENGE_BACKWARD_TO_FORWARD_HOLD_TIME_US = 130 * 1000;
// B→F反転時の前進開始速度。急な正逆転トルクを抑えるため低速から始める。
const int CHALLENGE_BACKWARD_TO_FORWARD_START_SPEED_DEG_S = 120;
// B→F反転時、前進速度を通常速度まで線形に上げる距離 [mm]。
const int CHALLENGE_BACKWARD_TO_FORWARD_ACCEL_DISTANCE_MM = 60;
// B→F反転1回で失われる固定距離を、前進前の低速後退で補う [mm]。
// B単独や旋回を挟むB→R→Fには加算しない。
const int CHALLENGE_BACKWARD_TO_FORWARD_BACKWARD_COMPENSATION_MM = 80;
// 上記の低速後退補正に使う速度 [deg/s]。
const int CHALLENGE_BACKWARD_TO_FORWARD_COMPENSATION_SPEED_DEG_S = 120;
// 前進側でバックラッシュの取り直しに消える固定距離を目標へ足す [mm]。
// Fの個数には比例させず、B→F反転1回につき一度だけ加算する。
const int CHALLENGE_BACKWARD_TO_FORWARD_FORWARD_COMPENSATION_MM = 80;
// 難所攻略のL/Rコマンドで、90度旋回に使う速度 [deg/s]。
// 140deg/sでも残った右旋回の惰性をさらに抑えるため、120deg/sへ下げた。
const int CHALLENGE_STEP_TURN_SPEED_DEG_S = 120;
// trueでは難所旋回の終了判定をジャイロでなく左右エンコーダで行う。
// ジャイロ値と強いブレーキの影響を切り分けるための現在の標準モード。
const bool USE_ENCODER_PRIMARY_CHALLENGE_TURN = true;
// エンコーダ理論角へ掛ける左右別の実機補正。
// 現在の右0.738は実走の右過旋回を抑える調整値。開始低速加速は別に無効化している。
const double CHALLENGE_ENCODER_LEFT_TURN_SCALE = 1.0;
const double CHALLENGE_ENCODER_RIGHT_TURN_SCALE = 0.738;
// 目標までこのエンコーダ角以内に入ったら、最低速度へ向けて線形減速する。
const double CHALLENGE_ENCODER_TURN_DECEL_WINDOW_DEG = 100.0;
// 減速中に静止摩擦で止まらないための最低モーター速度 [deg/s]。
const int CHALLENGE_ENCODER_TURN_MIN_SPEED_DEG_S = 35;
// 旋回中の左右エンコーダ移動量差へ掛ける同期補正ゲイン [(deg/s)/deg]。
// 正の左右差では先行する左輪を遅くし、遅れている右輪を相対的に速くする。
const double CHALLENGE_ENCODER_TURN_SYNC_KP = 0.8;
// 同期補正が旋回速度を急変させないための速度補正上限 [deg/s]。
const int CHALLENGE_ENCODER_TURN_SYNC_MAX_DEG_S = 40;
// 目標到達直後は惰性停止させ、速度が落ちてからブレーキを掛ける。
const int CHALLENGE_ENCODER_TURN_COAST_TIME_US = 30 * 1000;
const int CHALLENGE_ENCODER_TURN_BRAKE_SETTLE_TIME_US = 60 * 1000;
// 開始ボタンを離した時の揺れが収まってから、最初の直進方位を確定する待ち時間。
const int CHALLENGE_START_SETTLE_TIME_US = 200 * 1000;

}  // namespace etrobo_app

#endif
