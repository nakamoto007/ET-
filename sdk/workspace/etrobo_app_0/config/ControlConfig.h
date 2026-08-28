#ifndef CONTROL_CONFIG_H
#define CONTROL_CONFIG_H

namespace etrobo_app {

// 制御ループの周期。10ms = 100Hz。dly_tsk() 用なのでマイクロ秒で持つ。
const int CONTROL_PERIOD_US = 10 * 1000;
// 制御周期を秒で表した値。速度や加速度などの計算に使う。
const double CONTROL_PERIOD_SEC = 0.01;
// 起動時のIMU取り付け角補正で平均するサンプル数。
const int CALIBRATION_SAMPLES = 100;
// falseではSPIKE側のheadingだけを使い、独自の時間比例ドリフト補正を止める。
// 10秒の2点測定がノイズを過大評価して方位基準を回していないか確認する設定。
const bool ENABLE_CUSTOM_HEADING_DRIFT_CORRECTION = false;
// 独自補正が有効な場合に、停止したままIMU headingの流れを測る時間。
const int IMU_DRIFT_CALIBRATION_TIME_US = 10 * 1000 * 1000;
// IMUが準備完了になるまで待つ最大リトライ回数。
const int IMU_READY_RETRIES = 100;
// モーター取得/初期化が安定するまで待つ最大リトライ回数。
const int MOTOR_SETUP_RETRIES = 50;
// 汎用制御ループの安全停止用最大周期数。
const int MAX_CONTROL_CYCLES = 3000;
// 距離走行処理の安全停止用最大周期数。
const int MAX_DRIVE_CYCLES = 10000;
// 旋回処理の安全停止用最大周期数。
const int TURN_TIMEOUT_CYCLES = 800;
// SPIKE-RTの速度制御で指定するモーター速度の上限 [deg/s]。
const int MOTOR_SPEED_LIMIT_DEG_S = 1000;
// 距離指定の加減速で、0速度付近に張り付かないための最低直進速度 [deg/s]。
const int MIN_STRAIGHT_SPEED_DEG_S = 120;
// 旋回時にモーターが動き続ける最低速度 [deg/s]。小さすぎる速度を防ぐ。
const int MIN_TURN_SPEED_DEG_S = 40;

// 通常旋回を終了する角度誤差。静止摩擦が大きい機体で微小な往復旋回を避ける。
const double TURN_APPROACH_TOLERANCE_DEG = 1.0;
// 通常旋回で目標付近に入った状態を完了とみなすまでの連続周期数。
const int TURN_APPROACH_STABLE_COUNT = 3;
// 左旋回の主旋回を理想角より何度手前でブレーキするか。
// 左旋回には系統的な行き過ぎが未確認なので、現在は補正しない。
const double LEFT_TURN_PRE_BRAKE_DEG = 0.0;
// 右旋回の主旋回を理想角より何度手前でブレーキするか。
// 4度補正後も残った時計回りの行き過ぎを受け、段階的に7度へ強めた。
const double RIGHT_TURN_PRE_BRAKE_DEG = 7.0;
// 停止後補正が目標へ到達したとみなす角度誤差。
const double GYRO_TOLERANCE_DEG = 1.5;
// 通常旋回の停止後、そのまま走行を続けられる角度誤差。
// この範囲ではモーターを再始動せず、残差は次の直進PIDで理想方位へ戻す。
const double TURN_SETTLED_ACCEPTANCE_TOLERANCE_DEG = 3.0;
// 精密補正は目標角度内に入ったら即停止し、停止後の再測定で最終判定する。
const int TURN_STABLE_COUNT = 1;
// 旋回停止後に許容外だった場合、1回だけ合わせ直す最大周期数。
const int TURN_SETTLED_CORRECTION_CYCLES = 100;
// 旋回停止後の合わせ直しで使う速度上限 [deg/s]。
const int TURN_SETTLED_CORRECTION_SPEED_DEG_S = 50;
// 静止摩擦とバックラッシュによる往復を避けるため、停止後補正は1回に限る。
const int TURN_SETTLED_CORRECTION_ATTEMPTS = 1;
// 精密補正時だけ使う最低旋回速度 [deg/s]。低すぎると静止摩擦で動かない。
const int TURN_FINE_CORRECTION_MIN_SPEED_DEG_S = 50;
// この角度より外側では連続補正し、止めすぎによる動き出し不良を防ぐ。
const double TURN_FINE_CORRECTION_PULSE_WINDOW_DEG = 0.5;
// 精密補正中、目標近くで角速度が残っている時は早めに止めて流れを待つ。
const double TURN_FINE_COAST_BRAKE_WINDOW_DEG = 3.0;
const double TURN_FINE_COAST_BRAKE_YAW_RATE_DEG_S = 5.0;
// 精密補正の目標近くだけ1パルスごとに止め、IMU値が落ち着いてから再測定する。
const int TURN_FINE_CORRECTION_SETTLE_TIME_US = 60 * 1000;
// 直進/旋回をブレーキ停止した後、姿勢が落ち着くまで待つ時間。
const int DRIVE_STOP_SETTLE_TIME_US = 100 * 1000;
// エンコーダ式の旋回/走行で、理論値より余裕を持たせる倍率。
const double ENCODER_LIMIT_MARGIN = 1.35;
// エンコーダ式の旋回/走行で、倍率に加えて足す余裕角度。
const double ENCODER_LIMIT_EXTRA_DEG = 30.0;
// 左旋回の角度補正倍率。実機が不足/過回転する場合に調整する。
const double LEFT_TURN_ANGLE_SCALE = 1.0;
// 右旋回の角度補正倍率。実機が不足/過回転する場合に調整する。
const double RIGHT_TURN_ANGLE_SCALE = 1.0;
// 旋回目標角度へ一律に足す補正角度。
const double TURN_ANGLE_OFFSET_DEG = 0.0;
// 直進PIDで、方位誤差を速度補正へ変換する比例ゲイン。
const double STRAIGHT_PID_KP = 12.0;
// 直進PIDで、方位誤差の累積を速度補正へ変換する積分ゲイン。
const double STRAIGHT_PID_KI = 0.0;
// 直進PIDで、方位誤差の変化量を速度補正へ変換する微分ゲイン。
const double STRAIGHT_PID_KD = 1.5;
// 直進PIDで、この角度以下の方位誤差は補正しない。小刻みな頭振りを抑える。
const double STRAIGHT_PID_DEADBAND_DEG = 0.15;
// 直進開始直後だけ速度を抑える周期数。旋回後の再スタートの揺れを抑える。
const int STRAIGHT_START_SPEED_LIMIT_CYCLES = 20;
// 直進開始直後の速度上限 [deg/s]。
const int STRAIGHT_START_SPEED_LIMIT_DEG_S = 180;
// 直進開始直後だけPID補正量をなだらかに立ち上げる周期数。
const int STRAIGHT_PID_CORRECTION_RAMP_CYCLES = 35;
// 直進開始直後のPID補正量上限 [deg/s]。
const double STRAIGHT_START_CORRECTION_LIMIT_DEG_S = 60.0;
// 直進PIDの積分項上限。KIを上げた時の積分暴走を防ぐ。
const double STRAIGHT_PID_INTEGRAL_LIMIT_DEG_SEC = 20.0;
// 直進PIDで左右速度へ足し引きする補正量の上限 [deg/s]。
const double STRAIGHT_PID_CORRECTION_LIMIT_DEG_S = 240.0;
// 旋回PIDで、角度誤差を旋回速度へ変換する比例ゲイン。
const double TURN_PID_KP = 8.0;
// 旋回PIDで、角度誤差の累積を旋回速度へ変換する積分ゲイン。
const double TURN_PID_KI = 0.0;
// 旋回PIDで、角度誤差の変化量を旋回速度へ変換する微分ゲイン。
const double TURN_PID_KD = 0.35;
// 旋回PIDの積分項上限。KIを使う時の積分暴走を防ぐ。
const double TURN_PID_INTEGRAL_LIMIT_DEG_SEC = 20.0;
// 旧直進補正ヘルパー用。通常の直進走行は上のPID設定を使う。
const double STRAIGHT_GYRO_CORRECTION_GAIN = STRAIGHT_PID_KP;
const double STRAIGHT_ENCODER_CORRECTION_GAIN = 6.0;
const double STRAIGHT_CORRECTION_SPEED_LIMIT_DEG_S =
  STRAIGHT_PID_CORRECTION_LIMIT_DEG_S;

}  // namespace etrobo_app

#endif
