#include "app.h"
#include "DriveController.h"
#include "DriveBase.h"
#include "RobotConfig.h"

#include <cmath>
#include <cstdint>

#include <spike/hub/imu.h>
#include <spike/pup/motor.h>

namespace {

int absoluteValue(int value)
{
  return value < 0 ? -value : value;
}

int minimumInt(int left, int right)
{
  return left < right ? left : right;
}

double minimumDouble(double left, double right)
{
  return left < right ? left : right;
}

double clampDouble(double value, double limit)
{
  const double absolute_limit = std::fabs(limit);
  if (value > absolute_limit) {
    return absolute_limit;
  }
  if (value < -absolute_limit) {
    return -absolute_limit;
  }
  return value;
}

struct HeadingPidState {
  double integral;
  double previous_error;
  bool has_previous_error;
};

struct TurnProgressState {
  bool has_previous_heading;
  double previous_heading;
  double continuous_delta_degrees;
};

struct StraightControlOutput {
  double current_heading;
  double heading_error;
  double correction;
  int left_speed_deg_s;
  int right_speed_deg_s;
};

double straight_target_heading = 0.0;
bool straight_target_heading_valid = false;
bool straight_start_damping_pending = false;
straight_debug_t straight_debug = {};
uint32_t straight_debug_update_count = 0;
turn_debug_t turn_debug = {};
uint32_t turn_debug_update_count = 0;

void publishStraightDebug(const straight_debug_t &debug)
{
  straight_debug_t updated_debug = debug;

  loc_cpu();
  updated_debug.update_count = ++straight_debug_update_count;
  straight_debug = updated_debug;
  unl_cpu();
}

void publishTurnDebug(const turn_debug_t &debug)
{
  turn_debug_t updated_debug = debug;

  loc_cpu();
  updated_debug.update_count = ++turn_debug_update_count;
  turn_debug = updated_debug;
  unl_cpu();
}

void setStraightPidTargetHeading(double heading)
{
  straight_target_heading = heading;
  straight_target_heading_valid = true;
}

void requestStraightStartDamping(void)
{
  straight_start_damping_pending = true;
}

bool consumeStraightStartDamping(void)
{
  const bool should_damp = straight_start_damping_pending;
  straight_start_damping_pending = false;
  return should_damp;
}

double getStraightPidTargetHeading(void)
{
  if (!straight_target_heading_valid) {
    setStraightPidTargetHeading(hub_imu_get_heading());
  }
  return straight_target_heading;
}

int applyMinimumTurnSpeed(int speed_deg_s)
{
  if (speed_deg_s < etrobo_app::MIN_TURN_SPEED_DEG_S) {
    return etrobo_app::MIN_TURN_SPEED_DEG_S;
  }
  return speed_deg_s;
}

double encoderDegreesForTurn(double robot_degrees)
{
  return std::fabs(robot_degrees) *
         etrobo_app::TREAD_MM / etrobo_app::WHEEL_DIAMETER_MM;
}

double turnAngleScale(etrobo_app::TurnDirection direction)
{
  if (direction == etrobo_app::TurnDirection::Left) {
    return etrobo_app::LEFT_TURN_ANGLE_SCALE;
  }
  return etrobo_app::RIGHT_TURN_ANGLE_SCALE;
}

double correctedTurnTargetDegrees(int degrees,
                                  etrobo_app::TurnDirection direction)
{
  const double target_degrees =
    std::fabs(static_cast<double>(degrees)) * turnAngleScale(direction) +
    etrobo_app::TURN_ANGLE_OFFSET_DEG;
  if (target_degrees < 0.0) {
    return 0.0;
  }
  return target_degrees;
}

// 主旋回だけを理想角より手前で終え、ブレーキ後の惰性で目標へ近づける。
// 停止後判定と精密補正は元の理想角を使うため、補正値ぶん常に不足することはない。
double turnApproachTargetDegrees(double ideal_target_degrees,
                                 int direction_sign)
{
  const double pre_brake_degrees =
    direction_sign > 0 ? etrobo_app::RIGHT_TURN_PRE_BRAKE_DEG :
                         etrobo_app::LEFT_TURN_PRE_BRAKE_DEG;
  const double approach_target_degrees =
    ideal_target_degrees - pre_brake_degrees;
  return approach_target_degrees > 0.0 ? approach_target_degrees : 0.0;
}

double encoderTurnScale(int direction_sign)
{
  return direction_sign > 0 ?
    etrobo_app::CHALLENGE_ENCODER_RIGHT_TURN_SCALE :
    etrobo_app::CHALLENGE_ENCODER_LEFT_TURN_SCALE;
}

double robotDegreesForEncoderTurn(double encoder_degrees)
{
  return encoder_degrees *
         etrobo_app::WHEEL_DIAMETER_MM / etrobo_app::TREAD_MM;
}

// エンコーダ残量が減速区間へ入ったら、最低速度まで連続的に落とす。
// 最後まで一定速度で回して強く止める方式より、停止時の車体揺れを小さくする。
int encoderTurnSpeed(int max_speed_deg_s, double remaining_encoder_degrees)
{
  const int min_speed =
    minimumInt(max_speed_deg_s,
               etrobo_app::CHALLENGE_ENCODER_TURN_MIN_SPEED_DEG_S);
  const double decel_window =
    etrobo_app::CHALLENGE_ENCODER_TURN_DECEL_WINDOW_DEG;
  if (decel_window <= 0.0 || remaining_encoder_degrees >= decel_window) {
    return max_speed_deg_s;
  }

  const double progress = remaining_encoder_degrees / decel_window;
  const double speed = min_speed + (max_speed_deg_s - min_speed) * progress;
  return etrobo_app::clampMotorSpeed(speed);
}

// 目標直後はモーターをcoastにして回転エネルギーを逃がし、速度が落ちてから
// ブレーキを掛ける。停止直後の強い制動による車体の揺り戻しを避ける。
void coastThenBrakeTurn(const etrobo_app::DriveMotors &motors)
{
  etrobo_app::stopDriveMotors(motors);
  dly_tsk(etrobo_app::CHALLENGE_ENCODER_TURN_COAST_TIME_US);
  etrobo_app::brakeMotors(motors);
  dly_tsk(etrobo_app::CHALLENGE_ENCODER_TURN_BRAKE_SETTLE_TIME_US);
}

double averageEncoderTravelDegrees(const etrobo_app::DriveMotors &motors,
                                   int32_t left_start,
                                   int32_t right_start)
{
  const int32_t left_delta = pup_motor_get_count(motors.left) - left_start;
  const int32_t right_delta = pup_motor_get_count(motors.right) - right_start;
  return (std::fabs(static_cast<double>(left_delta)) +
          std::fabs(static_cast<double>(right_delta))) / 2.0;
}

struct EncoderTurnTravel {
  double left_degrees;
  double right_degrees;
  double average_degrees;
};

struct EncoderTurnWheelSpeeds {
  int left_speed_deg_s;
  int right_speed_deg_s;
  double sync_error_degrees;
};

// 旋回中の左右移動量を同じ時点で読み、平均値と左右差をログへ残せる形にする。
// 平均値だけでは片輪の滑りや引っ掛かりを見分けられないため、個別値も保持する。
EncoderTurnTravel readEncoderTurnTravel(
    const etrobo_app::DriveMotors &motors,
    int32_t left_start,
    int32_t right_start)
{
  EncoderTurnTravel travel = {};
  travel.left_degrees = std::fabs(static_cast<double>(
    pup_motor_get_count(motors.left) - left_start));
  travel.right_degrees = std::fabs(static_cast<double>(
    pup_motor_get_count(motors.right) - right_start));
  travel.average_degrees =
    (travel.left_degrees + travel.right_degrees) / 2.0;
  return travel;
}

// 目標未到達の車輪には静止摩擦を越える最低速度を保証し、到達済みなら止める。
// 左右を平均だけで止めず、片輪ずつ目標へ到達させるために使用する。
int clampEncoderTurnWheelSpeed(int requested_speed_deg_s,
                               int max_speed_deg_s,
                               bool target_reached)
{
  if (target_reached) {
    return 0;
  }

  const int min_speed =
    minimumInt(max_speed_deg_s,
               etrobo_app::CHALLENGE_ENCODER_TURN_MIN_SPEED_DEG_S);
  if (requested_speed_deg_s < min_speed) {
    return min_speed;
  }
  if (requested_speed_deg_s > max_speed_deg_s) {
    return max_speed_deg_s;
  }
  return requested_speed_deg_s;
}

// 左右エンコーダ移動量の差を速度差へ変換する。
// 先行輪を減速して遅れ輪を追い付かせ、旋回中の車体中心移動を抑える。
EncoderTurnWheelSpeeds synchronizedEncoderTurnWheelSpeeds(
    int base_speed_deg_s,
    int max_speed_deg_s,
    double target_encoder_degrees,
    const EncoderTurnTravel &travel)
{
  EncoderTurnWheelSpeeds speeds = {};
  speeds.sync_error_degrees =
    travel.left_degrees - travel.right_degrees;
  const double correction = clampDouble(
    speeds.sync_error_degrees *
      etrobo_app::CHALLENGE_ENCODER_TURN_SYNC_KP,
    etrobo_app::CHALLENGE_ENCODER_TURN_SYNC_MAX_DEG_S);
  const bool left_target_reached =
    travel.left_degrees >= target_encoder_degrees;
  const bool right_target_reached =
    travel.right_degrees >= target_encoder_degrees;
  speeds.left_speed_deg_s = clampEncoderTurnWheelSpeed(
    static_cast<int>(base_speed_deg_s - correction),
    max_speed_deg_s,
    left_target_reached);
  speeds.right_speed_deg_s = clampEncoderTurnWheelSpeed(
    static_cast<int>(base_speed_deg_s + correction),
    max_speed_deg_s,
    right_target_reached);
  return speeds;
}

void setSynchronizedEncoderTurnSpeeds(
    const etrobo_app::DriveMotors &motors,
    int direction_sign,
    const EncoderTurnWheelSpeeds &speeds)
{
  etrobo_app::setMotorSpeeds(
    motors,
    direction_sign * speeds.left_speed_deg_s,
    -direction_sign * speeds.right_speed_deg_s);
}

int signFromDistance(int distance)
{
  if (distance < 0) {
    return -1;
  }
  return 1;
}

int signFromDouble(double value)
{
  if (value < 0.0) {
    return -1;
  }
  return 1;
}

int interpolateSpeed(int start_speed_deg_s, int end_speed_deg_s,
                     double progress)
{
  const double speed_deg_s =
    start_speed_deg_s +
    (end_speed_deg_s - start_speed_deg_s) * progress;
  return etrobo_app::clampMotorSpeed(speed_deg_s);
}

int applyMinimumStraightSpeed(int speed_deg_s,
                              int start_speed_deg_s,
                              int end_speed_deg_s)
{
  if (start_speed_deg_s == 0 && end_speed_deg_s == 0) {
    return 0;
  }
  if (speed_deg_s >= etrobo_app::MIN_STRAIGHT_SPEED_DEG_S) {
    return speed_deg_s;
  }
  return etrobo_app::MIN_STRAIGHT_SPEED_DEG_S;
}

double angleError(double target_heading, double current_heading)
{
  double error = target_heading - current_heading;
  while (error > 180.0) {
    error -= 360.0;
  }
  while (error < -180.0) {
    error += 360.0;
  }
  return error;
}

double headingStepDelta(double current_heading, double previous_heading)
{
  double delta = current_heading - previous_heading;
  while (delta > 180.0) {
    delta -= 360.0;
  }
  while (delta < -180.0) {
    delta += 360.0;
  }
  return delta;
}

double updateTurnProgress(TurnProgressState *progress,
                          double current_heading)
{
  if (!progress->has_previous_heading) {
    progress->has_previous_heading = true;
    progress->previous_heading = current_heading;
    return progress->continuous_delta_degrees;
  }

  progress->continuous_delta_degrees +=
    headingStepDelta(current_heading, progress->previous_heading);
  progress->previous_heading = current_heading;
  return progress->continuous_delta_degrees;
}

double turnProgressError(TurnProgressState *progress,
                         double current_heading,
                         double target_degrees,
                         int direction_sign)
{
  const double continuous_delta =
    updateTurnProgress(progress, current_heading);
  const double directed_travelled = direction_sign * continuous_delta;
  return direction_sign * (target_degrees - directed_travelled);
}

double applyDeadband(double value, double deadband)
{
  if (std::fabs(value) <= deadband) {
    return 0.0;
  }
  return value;
}

double updateStraightPid(HeadingPidState *pid, double error)
{
  pid->integral += error * etrobo_app::CONTROL_PERIOD_SEC;
  pid->integral = clampDouble(
    pid->integral, etrobo_app::STRAIGHT_PID_INTEGRAL_LIMIT_DEG_SEC);

  double derivative = 0.0;
  if (pid->has_previous_error) {
    derivative =
      (error - pid->previous_error) / etrobo_app::CONTROL_PERIOD_SEC;
  } else {
    pid->has_previous_error = true;
  }
  pid->previous_error = error;

  const double correction =
    etrobo_app::STRAIGHT_PID_KP * error +
    etrobo_app::STRAIGHT_PID_KI * pid->integral +
    etrobo_app::STRAIGHT_PID_KD * derivative;

  return clampDouble(correction,
                     etrobo_app::STRAIGHT_PID_CORRECTION_LIMIT_DEG_S);
}

double updateTurnPid(HeadingPidState *pid, double error)
{
  pid->integral += error * etrobo_app::CONTROL_PERIOD_SEC;
  pid->integral = clampDouble(
    pid->integral, etrobo_app::TURN_PID_INTEGRAL_LIMIT_DEG_SEC);

  double derivative = 0.0;
  if (pid->has_previous_error) {
    derivative =
      (error - pid->previous_error) / etrobo_app::CONTROL_PERIOD_SEC;
  } else {
    pid->has_previous_error = true;
  }
  pid->previous_error = error;

  const double speed =
    etrobo_app::TURN_PID_KP * error +
    etrobo_app::TURN_PID_KI * pid->integral +
    etrobo_app::TURN_PID_KD * derivative;

  return clampDouble(speed, etrobo_app::MOTOR_SPEED_LIMIT_DEG_S);
}

int applyStraightStartSpeedLimit(int speed_deg_s, int cycle)
{
  if (cycle >= etrobo_app::STRAIGHT_START_SPEED_LIMIT_CYCLES) {
    return speed_deg_s;
  }

  const int speed_abs = absoluteValue(speed_deg_s);
  if (speed_abs <= etrobo_app::STRAIGHT_START_SPEED_LIMIT_DEG_S) {
    return speed_deg_s;
  }

  return signFromDouble(static_cast<double>(speed_deg_s)) *
         etrobo_app::STRAIGHT_START_SPEED_LIMIT_DEG_S;
}

double straightStartCorrectionLimit(int base_speed_deg_s, int cycle)
{
  const double target_limit =
    minimumDouble(std::fabs(static_cast<double>(base_speed_deg_s)),
                  etrobo_app::STRAIGHT_PID_CORRECTION_LIMIT_DEG_S);
  if (cycle >= etrobo_app::STRAIGHT_PID_CORRECTION_RAMP_CYCLES ||
      target_limit <= etrobo_app::STRAIGHT_START_CORRECTION_LIMIT_DEG_S) {
    return target_limit;
  }

  const double progress =
    static_cast<double>(cycle + 1) /
    static_cast<double>(etrobo_app::STRAIGHT_PID_CORRECTION_RAMP_CYCLES);
  return etrobo_app::STRAIGHT_START_CORRECTION_LIMIT_DEG_S +
         (target_limit - etrobo_app::STRAIGHT_START_CORRECTION_LIMIT_DEG_S) *
         progress;
}

StraightControlOutput setPidStraightSpeed(
    const etrobo_app::DriveMotors &motors,
    int base_speed_deg_s,
    double target_heading,
    HeadingPidState *pid,
    double correction_limit_deg_s)
{
  StraightControlOutput output = {};
  output.current_heading = hub_imu_get_heading();
  output.heading_error =
    applyDeadband(angleError(target_heading, output.current_heading),
                  etrobo_app::STRAIGHT_PID_DEADBAND_DEG);
  output.correction =
    clampDouble(updateStraightPid(pid, output.heading_error),
                minimumDouble(std::fabs(static_cast<double>(base_speed_deg_s)),
                              correction_limit_deg_s));
  output.left_speed_deg_s =
    etrobo_app::clampMotorSpeed(base_speed_deg_s + output.correction);
  output.right_speed_deg_s =
    etrobo_app::clampMotorSpeed(base_speed_deg_s - output.correction);

  etrobo_app::setMotorSpeeds(motors,
                             output.left_speed_deg_s,
                             output.right_speed_deg_s);
  return output;
}

void brakeAndSettle(const etrobo_app::DriveMotors &motors)
{
  etrobo_app::brakeMotors(motors);
  dly_tsk(etrobo_app::DRIVE_STOP_SETTLE_TIME_US);
}

drive_result_t driveStraightByEncoder(int start_speed_deg_s,
                                      int end_speed_deg_s,
                                      int encoder_degrees,
                                      bool brake_at_end)
{
  etrobo_app::DriveMotors motors;
  if (!etrobo_app::getDriveMotors(&motors)) {
    return DRIVE_RESULT_MOTOR_ERROR;
  }

  const int target_degrees = absoluteValue(encoder_degrees);
  if (target_degrees == 0) {
    etrobo_app::brakeMotors(motors);
    return DRIVE_RESULT_OK;
  }

  const int direction = signFromDistance(encoder_degrees);
  const int start_speed_abs = absoluteValue(start_speed_deg_s);
  const int end_speed_abs = absoluteValue(end_speed_deg_s);
  const int32_t left_start = pup_motor_get_count(motors.left);
  const int32_t right_start = pup_motor_get_count(motors.right);
  const double target_heading = getStraightPidTargetHeading();
  const bool damp_straight_start = consumeStraightStartDamping();
  HeadingPidState straight_pid = {0.0, 0.0, false};
  straight_debug_t debug = {};
  debug.active = true;
  debug.cycle = 0;
  debug.base_speed_deg_s = 0;
  debug.left_speed_deg_s = 0;
  debug.right_speed_deg_s = 0;
  debug.result = DRIVE_RESULT_OK;
  debug.target_heading = target_heading;
  debug.current_heading = hub_imu_get_heading();
  debug.heading_error = angleError(target_heading, debug.current_heading);
  debug.correction_deg_s = 0.0;
  debug.correction_limit_deg_s = 0.0;
  debug.travelled_degrees = 0.0;
  debug.target_degrees = target_degrees;
  publishStraightDebug(debug);

  for (int cycle = 0; cycle < etrobo_app::MAX_DRIVE_CYCLES; ++cycle) {
    const double travelled_degrees =
      averageEncoderTravelDegrees(motors, left_start, right_start);
    if (travelled_degrees >= target_degrees) {
      if (brake_at_end) {
        brakeAndSettle(motors);
      }
      debug.active = false;
      debug.cycle = cycle;
      debug.base_speed_deg_s = 0;
      debug.left_speed_deg_s = 0;
      debug.right_speed_deg_s = 0;
      debug.result = DRIVE_RESULT_OK;
      debug.current_heading = hub_imu_get_heading();
      debug.heading_error = angleError(target_heading, debug.current_heading);
      debug.correction_deg_s = 0.0;
      debug.correction_limit_deg_s = 0.0;
      debug.travelled_degrees = travelled_degrees;
      publishStraightDebug(debug);
      return DRIVE_RESULT_OK;
    }

    const double progress = travelled_degrees / target_degrees;
    const int interpolated_speed =
      interpolateSpeed(start_speed_abs, end_speed_abs, progress);
    const int speed_deg_s =
      direction * applyMinimumStraightSpeed(interpolated_speed,
                                            start_speed_abs,
                                            end_speed_abs);
    const int limited_speed_deg_s =
      damp_straight_start ? applyStraightStartSpeedLimit(speed_deg_s, cycle) :
                            speed_deg_s;
    const double correction_limit_deg_s =
      damp_straight_start ?
      straightStartCorrectionLimit(limited_speed_deg_s, cycle) :
      etrobo_app::STRAIGHT_PID_CORRECTION_LIMIT_DEG_S;
    const StraightControlOutput output =
      setPidStraightSpeed(motors,
                          limited_speed_deg_s,
                          target_heading,
                          &straight_pid,
                          correction_limit_deg_s);
    debug.active = true;
    debug.cycle = cycle;
    debug.base_speed_deg_s = limited_speed_deg_s;
    debug.left_speed_deg_s = output.left_speed_deg_s;
    debug.right_speed_deg_s = output.right_speed_deg_s;
    debug.result = DRIVE_RESULT_OK;
    debug.current_heading = output.current_heading;
    debug.heading_error = output.heading_error;
    debug.correction_deg_s = output.correction;
    debug.correction_limit_deg_s = correction_limit_deg_s;
    debug.travelled_degrees = travelled_degrees;
    publishStraightDebug(debug);
    dly_tsk(etrobo_app::CONTROL_PERIOD_US);
  }

  brakeAndSettle(motors);
  debug.active = false;
  debug.result = DRIVE_RESULT_TIMEOUT;
  debug.base_speed_deg_s = 0;
  debug.left_speed_deg_s = 0;
  debug.right_speed_deg_s = 0;
  debug.current_heading = hub_imu_get_heading();
  debug.heading_error = angleError(target_heading, debug.current_heading);
  debug.correction_deg_s = 0.0;
  debug.correction_limit_deg_s = 0.0;
  debug.travelled_degrees =
    averageEncoderTravelDegrees(motors, left_start, right_start);
  publishStraightDebug(debug);
  return DRIVE_RESULT_TIMEOUT;
}

drive_result_t driveCurveByEncoder(int left_speed_deg_s,
                                   int right_speed_deg_s,
                                   int encoder_degrees,
                                   bool brake_at_end)
{
  etrobo_app::DriveMotors motors;
  if (!etrobo_app::getDriveMotors(&motors)) {
    return DRIVE_RESULT_MOTOR_ERROR;
  }

  const int target_degrees = absoluteValue(encoder_degrees);
  if (target_degrees == 0) {
    if (brake_at_end) {
      etrobo_app::brakeMotors(motors);
    }
    return DRIVE_RESULT_OK;
  }

  const int direction = signFromDistance(encoder_degrees);
  const int left_speed =
    direction * etrobo_app::clampMotorSpeed(absoluteValue(left_speed_deg_s));
  const int right_speed =
    direction * etrobo_app::clampMotorSpeed(absoluteValue(right_speed_deg_s));
  const int32_t left_start = pup_motor_get_count(motors.left);
  const int32_t right_start = pup_motor_get_count(motors.right);

  for (int cycle = 0; cycle < etrobo_app::MAX_DRIVE_CYCLES; ++cycle) {
    if (averageEncoderTravelDegrees(motors, left_start, right_start) >=
        target_degrees) {
      if (brake_at_end) {
        brakeAndSettle(motors);
      }
      return DRIVE_RESULT_OK;
    }

    etrobo_app::setMotorSpeeds(motors, left_speed, right_speed);
    dly_tsk(etrobo_app::CONTROL_PERIOD_US);
  }

  brakeAndSettle(motors);
  return DRIVE_RESULT_TIMEOUT;
}

int turnSign(etrobo_app::TurnDirection direction)
{
  return direction == etrobo_app::TurnDirection::Left ? -1 : 1;
}

int calculateTurnPidSpeed(HeadingPidState *pid,
                          double heading_error,
                          int max_speed_deg_s,
                          double tolerance_deg,
                          int min_speed_deg_s,
                          bool force_error_direction)
{
  if (std::fabs(heading_error) <= tolerance_deg) {
    return 0;
  }

  const double pid_speed =
    clampDouble(updateTurnPid(pid, heading_error),
                static_cast<double>(max_speed_deg_s));
  int speed_sign = signFromDouble(pid_speed);
  double speed_abs = std::fabs(pid_speed);
  if (force_error_direction) {
    speed_sign = signFromDouble(heading_error);
  }
  if (speed_abs < 1.0) {
    speed_sign = signFromDouble(heading_error);
  }
  if (speed_abs < min_speed_deg_s) {
    speed_abs = min_speed_deg_s;
  }
  if (speed_abs > max_speed_deg_s) {
    speed_abs = max_speed_deg_s;
  }

  return speed_sign * static_cast<int>(speed_abs);
}

void setSignedTurnSpeed(const etrobo_app::DriveMotors &motors,
                        int turn_speed_deg_s)
{
  etrobo_app::setMotorSpeeds(motors, turn_speed_deg_s, -turn_speed_deg_s);
}

double getYawRateDegreesPerSecond(void)
{
  float angular_velocity[3] = {0.0F, 0.0F, 0.0F};
  hub_imu_get_angular_velocity(angular_velocity);
  return static_cast<double>(angular_velocity[2]);
}

int runTurnPidUntilStable(const etrobo_app::DriveMotors &motors,
                          double target_degrees,
                          int direction_sign,
                          int max_speed_deg_s,
                          int32_t left_start,
                          int32_t right_start,
                          double encoder_limit,
                          int timeout_cycles,
                          int phase,
                          double tolerance_deg,
                          int stable_required_count,
                          int min_speed_deg_s,
                          bool force_error_direction,
                          TurnProgressState *progress,
                          turn_debug_t *debug)
{
  int stable_count = 0;
  HeadingPidState turn_pid = {0.0, 0.0, false};
  bool has_previous_heading_error = false;
  double previous_heading_error = 0.0;
  for (int cycle = 0; cycle < timeout_cycles; ++cycle) {
    const double encoder_degrees =
      averageEncoderTravelDegrees(motors, left_start, right_start);
    const double current_heading = hub_imu_get_heading();
    const double heading_error =
      turnProgressError(progress,
                        current_heading,
                        target_degrees,
                        direction_sign);
    int turn_speed = 0;

    if (std::fabs(heading_error) <= tolerance_deg) {
      ++stable_count;
      debug->active = true;
      debug->phase = phase;
      debug->current_heading = current_heading;
      debug->heading_error = heading_error;
      debug->encoder_degrees = encoder_degrees;
      debug->turn_speed_deg_s = 0;
      debug->stable_count = stable_count;
      debug->result = TURN_RESULT_OK;
      publishTurnDebug(*debug);
      etrobo_app::brakeMotors(motors);

      if (stable_count >= stable_required_count) {
        return TURN_RESULT_OK;
      }
      dly_tsk(etrobo_app::CONTROL_PERIOD_US);
      continue;
    } else {
      stable_count = 0;
    }

    if (encoder_degrees >= encoder_limit) {
      debug->active = true;
      debug->phase = phase;
      debug->current_heading = current_heading;
      debug->heading_error = heading_error;
      debug->encoder_degrees = encoder_degrees;
      debug->turn_speed_deg_s = 0;
      debug->stable_count = stable_count;
      debug->result = TURN_RESULT_ENCODER_LIMIT;
      publishTurnDebug(*debug);
      return TURN_RESULT_ENCODER_LIMIT;
    }

    const double heading_error_abs = std::fabs(heading_error);
    const double previous_heading_error_abs =
      std::fabs(previous_heading_error);
    const bool should_brake_for_coast =
      force_error_direction &&
      has_previous_heading_error &&
      heading_error_abs <= etrobo_app::TURN_FINE_COAST_BRAKE_WINDOW_DEG &&
      heading_error_abs < previous_heading_error_abs &&
      std::fabs(getYawRateDegreesPerSecond()) >=
        etrobo_app::TURN_FINE_COAST_BRAKE_YAW_RATE_DEG_S;
    if (should_brake_for_coast) {
      debug->active = true;
      debug->phase = phase;
      debug->current_heading = current_heading;
      debug->heading_error = heading_error;
      debug->encoder_degrees = encoder_degrees;
      debug->turn_speed_deg_s = 0;
      debug->stable_count = stable_count;
      debug->result = TURN_RESULT_OK;
      publishTurnDebug(*debug);
      etrobo_app::brakeMotors(motors);
      dly_tsk(etrobo_app::TURN_FINE_CORRECTION_SETTLE_TIME_US);
      previous_heading_error = heading_error;
      continue;
    }

    turn_speed =
      calculateTurnPidSpeed(&turn_pid,
                            heading_error,
                            max_speed_deg_s,
                            tolerance_deg,
                            min_speed_deg_s,
                            force_error_direction);
    debug->active = true;
    debug->phase = phase;
    debug->current_heading = current_heading;
    debug->heading_error = heading_error;
    debug->encoder_degrees = encoder_degrees;
    debug->turn_speed_deg_s = turn_speed;
    debug->stable_count = stable_count;
    debug->result = TURN_RESULT_OK;
    publishTurnDebug(*debug);
    setSignedTurnSpeed(motors, turn_speed);
    dly_tsk(etrobo_app::CONTROL_PERIOD_US);
    const bool pulse_fine_correction =
      force_error_direction &&
      std::fabs(heading_error) <=
        etrobo_app::TURN_FINE_CORRECTION_PULSE_WINDOW_DEG;
    if (pulse_fine_correction) {
      etrobo_app::brakeMotors(motors);
      dly_tsk(etrobo_app::TURN_FINE_CORRECTION_SETTLE_TIME_US);
    }
    previous_heading_error = heading_error;
    has_previous_heading_error = true;
  }

  debug->active = true;
  debug->phase = phase;
  debug->current_heading = hub_imu_get_heading();
  debug->heading_error =
    turnProgressError(progress,
                      debug->current_heading,
                      target_degrees,
                      direction_sign);
  debug->encoder_degrees =
    averageEncoderTravelDegrees(motors, left_start, right_start);
  debug->turn_speed_deg_s = 0;
  debug->stable_count = stable_count;
  debug->result = TURN_RESULT_TIMEOUT;
  publishTurnDebug(*debug);
  return TURN_RESULT_TIMEOUT;
}

// 相対旋回と絶対方位旋回で共有する旋回制御本体。
// target_degreesは開始時点から実際に回す角度、target_headingは成功後に
// 直進PIDへ引き継ぐ絶対方位として役割を分けている。
int runTurnControl(const etrobo_app::DriveMotors &motors,
                   int speed,
                   int command_degrees,
                   double start_heading,
                   double target_heading,
                   double target_degrees,
                   int direction_sign)
{
  const int base_speed =
    applyMinimumTurnSpeed(
      etrobo_app::clampMotorSpeed(absoluteValue(speed)));
  const int32_t left_start = pup_motor_get_count(motors.left);
  const int32_t right_start = pup_motor_get_count(motors.right);
  const double approach_target_degrees =
    turnApproachTargetDegrees(target_degrees, direction_sign);
  const double encoder_limit =
    encoderDegreesForTurn(target_degrees) * etrobo_app::ENCODER_LIMIT_MARGIN +
    etrobo_app::ENCODER_LIMIT_EXTRA_DEG;
  turn_debug_t debug = {};
  debug.active = true;
  debug.phase = 1;
  debug.command_degrees = command_degrees;
  debug.direction = direction_sign;
  debug.max_speed_deg_s = base_speed;
  debug.result = TURN_RESULT_OK;
  debug.start_heading = start_heading;
  debug.target_degrees = target_degrees;
  debug.approach_target_degrees = approach_target_degrees;
  debug.target_heading = target_heading;
  debug.current_heading = start_heading;
  debug.heading_error = direction_sign * approach_target_degrees;
  debug.encoder_limit_degrees = encoder_limit;
  publishTurnDebug(debug);

  TurnProgressState turn_progress = {false, 0.0, 0.0};
  int result = runTurnPidUntilStable(motors,
                                     approach_target_degrees,
                                     direction_sign,
                                     base_speed,
                                     left_start,
                                     right_start,
                                     encoder_limit,
                                     etrobo_app::TURN_TIMEOUT_CYCLES,
                                     1,
                                     etrobo_app::TURN_APPROACH_TOLERANCE_DEG,
                                     etrobo_app::TURN_APPROACH_STABLE_COUNT,
                                     etrobo_app::MIN_TURN_SPEED_DEG_S,
                                     false,
                                     &turn_progress,
                                     &debug);
  brakeAndSettle(motors);

  double settled_error =
    turnProgressError(&turn_progress,
                      hub_imu_get_heading(),
                      target_degrees,
                      direction_sign);
  const bool approach_hard_failure =
    result == TURN_RESULT_MOTOR_ERROR || result == TURN_RESULT_ENCODER_LIMIT;
  if (!approach_hard_failure &&
      std::fabs(settled_error) <=
        etrobo_app::TURN_SETTLED_ACCEPTANCE_TOLERANCE_DEG) {
    result = TURN_RESULT_OK;
  }

  // 走行継続可能な誤差内ではモーターを再始動しない。許容外の場合だけ
  // 設定回数まで再旋回し、難所の小さな残差は続く直進PIDへ任せる。
  for (int attempt = 0;
       attempt < etrobo_app::TURN_SETTLED_CORRECTION_ATTEMPTS &&
       result != TURN_RESULT_MOTOR_ERROR &&
       result != TURN_RESULT_ENCODER_LIMIT &&
       std::fabs(settled_error) >
         etrobo_app::TURN_SETTLED_ACCEPTANCE_TOLERANCE_DEG;
       ++attempt) {
    const int correction_speed =
      minimumInt(base_speed, etrobo_app::TURN_SETTLED_CORRECTION_SPEED_DEG_S);
    result = runTurnPidUntilStable(
      motors,
      target_degrees,
      direction_sign,
      correction_speed,
      left_start,
      right_start,
      encoder_limit,
      etrobo_app::TURN_SETTLED_CORRECTION_CYCLES,
      2 + attempt,
      etrobo_app::GYRO_TOLERANCE_DEG,
      etrobo_app::TURN_STABLE_COUNT,
      etrobo_app::TURN_FINE_CORRECTION_MIN_SPEED_DEG_S,
      true,
      &turn_progress,
      &debug);
    brakeAndSettle(motors);

    settled_error =
      turnProgressError(&turn_progress,
                        hub_imu_get_heading(),
                        target_degrees,
                        direction_sign);
  }

  // 制御ループのTIMEOUTでも停止位置が実用範囲なら走行を継続する。
  // モーター異常とエンコーダ上限は安全上の失敗なので上書きしない。
  const bool final_hard_failure =
    result == TURN_RESULT_MOTOR_ERROR || result == TURN_RESULT_ENCODER_LIMIT;
  if (!final_hard_failure) {
    result = std::fabs(settled_error) <=
               etrobo_app::TURN_SETTLED_ACCEPTANCE_TOLERANCE_DEG ?
             TURN_RESULT_OK : TURN_RESULT_TIMEOUT;
  }

  debug.active = false;
  debug.current_heading = hub_imu_get_heading();
  debug.heading_error =
    turnProgressError(&turn_progress,
                      debug.current_heading,
                      target_degrees,
                      direction_sign);
  debug.encoder_degrees =
    averageEncoderTravelDegrees(motors, left_start, right_start);
  debug.turn_speed_deg_s = 0;
  debug.result = result;
  publishTurnDebug(debug);

  if (result == TURN_RESULT_OK) {
    setStraightPidTargetHeading(target_heading);
    requestStraightStartDamping();
  } else {
    reset_straight_pid_heading();
  }
  return result;
}

}  // namespace

void stop_motors(void)
{
  etrobo_app::DriveMotors motors;
  if (etrobo_app::getDriveMotors(&motors)) {
    etrobo_app::brakeMotors(motors);
  }
}

void hold_motors(void)
{
  etrobo_app::DriveMotors motors;
  if (etrobo_app::getDriveMotors(&motors)) {
    // brakeは受動制動だが、holdは呼出時のエンコーダ位置を能動的に維持する。
    // 後退停止直後に機体が前へ戻り、実後退距離が短くなる現象を抑える。
    pup_motor_hold(motors.left);
    pup_motor_hold(motors.right);
  }
}

void reset_straight_pid_heading(void)
{
  setStraightPidTargetHeading(hub_imu_get_heading());
}

drive_result_t drive_straight_mm(int speed, int distance_mm)
{
  const int encoder_degrees =
    static_cast<int>(etrobo_app::mmToEncoderDegrees(distance_mm));
  return driveStraightByEncoder(speed, speed, encoder_degrees, true);
}

drive_result_t drive_straight_mm_keep_speed(int speed, int distance_mm)
{
  const int encoder_degrees =
    static_cast<int>(etrobo_app::mmToEncoderDegrees(distance_mm));
  return driveStraightByEncoder(speed, speed, encoder_degrees, false);
}

drive_result_t drive_curve_mm(int left_speed, int right_speed,
                              int distance_mm)
{
  const int encoder_degrees =
    static_cast<int>(etrobo_app::mmToEncoderDegrees(distance_mm));
  return driveCurveByEncoder(left_speed, right_speed, encoder_degrees, true);
}

drive_result_t drive_curve_mm_keep_speed(int left_speed, int right_speed,
                                         int distance_mm)
{
  const int encoder_degrees =
    static_cast<int>(etrobo_app::mmToEncoderDegrees(distance_mm));
  return driveCurveByEncoder(left_speed, right_speed, encoder_degrees, false);
}

drive_result_t speed_up(int start_speed, int end_speed, int distance_mm)
{
  const int encoder_degrees =
    static_cast<int>(etrobo_app::mmToEncoderDegrees(distance_mm));
  return driveStraightByEncoder(start_speed, end_speed, encoder_degrees, false);
}

drive_result_t speed_down(int start_speed, int end_speed, int distance_mm)
{
  const int encoder_degrees =
    static_cast<int>(etrobo_app::mmToEncoderDegrees(distance_mm));
  const bool should_brake = end_speed == 0;
  return driveStraightByEncoder(start_speed, end_speed, encoder_degrees,
                                should_brake);
}

int turn(int speed, int degrees)
{
  const etrobo_app::TurnDirection direction =
    degrees < 0 ? etrobo_app::TurnDirection::Left :
                  etrobo_app::TurnDirection::Right;
  etrobo_app::DriveMotors motors;
  if (!etrobo_app::getDriveMotors(&motors)) {
    return TURN_RESULT_MOTOR_ERROR;
  }

  const double target_degrees =
    correctedTurnTargetDegrees(degrees, direction);
  if (target_degrees <= 0.0) {
    reset_straight_pid_heading();
    return TURN_RESULT_OK;
  }

  const double start_heading = hub_imu_get_heading();
  const double target_heading =
    start_heading + turnSign(direction) * target_degrees;
  const int direction_sign = turnSign(direction);
  return runTurnControl(motors,
                        speed,
                        degrees,
                        start_heading,
                        target_heading,
                        target_degrees,
                        direction_sign);
}

int turn_by_encoder(int speed, int degrees)
{
  etrobo_app::DriveMotors motors;
  if (!etrobo_app::getDriveMotors(&motors)) {
    return TURN_RESULT_MOTOR_ERROR;
  }

  const int direction_sign = degrees < 0 ? -1 : 1;
  const double ideal_robot_degrees =
    std::fabs(static_cast<double>(degrees));
  if (ideal_robot_degrees <= 0.0) {
    reset_straight_pid_heading();
    return TURN_RESULT_OK;
  }

  const int max_speed =
    applyMinimumTurnSpeed(
      etrobo_app::clampMotorSpeed(absoluteValue(speed)));
  const double encoder_scale = encoderTurnScale(direction_sign);
  const double target_encoder_degrees =
    encoderDegreesForTurn(ideal_robot_degrees) * encoder_scale;
  const double encoder_limit =
    target_encoder_degrees * etrobo_app::ENCODER_LIMIT_MARGIN +
    etrobo_app::ENCODER_LIMIT_EXTRA_DEG;
  const int32_t left_start = pup_motor_get_count(motors.left);
  const int32_t right_start = pup_motor_get_count(motors.right);
  const double start_heading = hub_imu_get_heading();

  turn_debug_t debug = {};
  debug.active = true;
  debug.phase = 10;
  debug.command_degrees = degrees;
  debug.direction = direction_sign;
  debug.max_speed_deg_s = max_speed;
  debug.result = TURN_RESULT_OK;
  debug.start_heading = start_heading;
  debug.target_degrees = ideal_robot_degrees;
  debug.approach_target_degrees = ideal_robot_degrees * encoder_scale;
  debug.target_heading = start_heading + direction_sign * ideal_robot_degrees;
  debug.current_heading = start_heading;
  debug.heading_error =
    direction_sign * debug.approach_target_degrees;
  debug.encoder_target_degrees = target_encoder_degrees;
  debug.encoder_limit_degrees = encoder_limit;
  publishTurnDebug(debug);

  for (int cycle = 0; cycle < etrobo_app::TURN_TIMEOUT_CYCLES; ++cycle) {
    const EncoderTurnTravel travel =
      readEncoderTurnTravel(motors, left_start, right_start);
    const double encoder_degrees = travel.average_degrees;
    const double remaining_encoder_degrees =
      target_encoder_degrees - encoder_degrees;
    const double left_remaining_encoder_degrees =
      target_encoder_degrees - travel.left_degrees;
    const double right_remaining_encoder_degrees =
      target_encoder_degrees - travel.right_degrees;
    const bool left_target_reached =
      left_remaining_encoder_degrees <= 0.0;
    const bool right_target_reached =
      right_remaining_encoder_degrees <= 0.0;

    // 平均が目標未満でも片輪だけが大きく回る故障・空転を安全上限で止める。
    if (travel.left_degrees >= encoder_limit ||
        travel.right_degrees >= encoder_limit) {
      debug.encoder_stop_degrees = encoder_degrees;
      coastThenBrakeTurn(motors);
      const EncoderTurnTravel final_travel =
        readEncoderTurnTravel(motors, left_start, right_start);
      debug.active = false;
      debug.current_heading = hub_imu_get_heading();
      debug.heading_error = direction_sign * robotDegreesForEncoderTurn(
        target_encoder_degrees - final_travel.average_degrees);
      debug.encoder_degrees = final_travel.average_degrees;
      debug.left_encoder_degrees = final_travel.left_degrees;
      debug.right_encoder_degrees = final_travel.right_degrees;
      debug.encoder_sync_error_degrees =
        final_travel.left_degrees - final_travel.right_degrees;
      debug.turn_speed_deg_s = 0;
      debug.left_turn_speed_deg_s = 0;
      debug.right_turn_speed_deg_s = 0;
      debug.result = TURN_RESULT_ENCODER_LIMIT;
      publishTurnDebug(debug);
      reset_straight_pid_heading();
      return TURN_RESULT_ENCODER_LIMIT;
    }

    if (left_target_reached && right_target_reached) {
      // 左右平均ではなく両輪の到達を確認し、片輪不足による中心移動を残さない。
      // 指令を切った時点と完全停止後を分け、coast中の惰性回転量も測定する。
      debug.encoder_stop_degrees = encoder_degrees;
      coastThenBrakeTurn(motors);
      const EncoderTurnTravel final_travel =
        readEncoderTurnTravel(motors, left_start, right_start);
      debug.active = false;
      debug.current_heading = hub_imu_get_heading();
      debug.heading_error = direction_sign * robotDegreesForEncoderTurn(
        target_encoder_degrees - final_travel.average_degrees);
      debug.encoder_degrees = final_travel.average_degrees;
      debug.left_encoder_degrees = final_travel.left_degrees;
      debug.right_encoder_degrees = final_travel.right_degrees;
      debug.encoder_sync_error_degrees =
        final_travel.left_degrees - final_travel.right_degrees;
      debug.turn_speed_deg_s = 0;
      debug.left_turn_speed_deg_s = 0;
      debug.right_turn_speed_deg_s = 0;
      debug.result = TURN_RESULT_OK;
      publishTurnDebug(debug);

      // 絶対ジャイロ方位へ戻そうとせず、旋回直後の値を次区間の短期保持方位にする。
      reset_straight_pid_heading();
      requestStraightStartDamping();
      return TURN_RESULT_OK;
    }

    // 遅れている側の残量で共通速度を減速し、同期補正で左右速度を分ける。
    const double profile_remaining_encoder_degrees =
      left_remaining_encoder_degrees > right_remaining_encoder_degrees ?
      left_remaining_encoder_degrees : right_remaining_encoder_degrees;
    const int turn_speed =
      encoderTurnSpeed(max_speed, profile_remaining_encoder_degrees);
    const EncoderTurnWheelSpeeds wheel_speeds =
      synchronizedEncoderTurnWheelSpeeds(
        turn_speed, max_speed, target_encoder_degrees, travel);
    debug.active = true;
    debug.current_heading = hub_imu_get_heading();
    debug.heading_error = direction_sign *
      robotDegreesForEncoderTurn(remaining_encoder_degrees);
    debug.encoder_degrees = encoder_degrees;
    debug.left_encoder_degrees = travel.left_degrees;
    debug.right_encoder_degrees = travel.right_degrees;
    debug.turn_speed_deg_s = direction_sign * turn_speed;
    debug.left_turn_speed_deg_s =
      direction_sign * wheel_speeds.left_speed_deg_s;
    debug.right_turn_speed_deg_s =
      -direction_sign * wheel_speeds.right_speed_deg_s;
    debug.encoder_sync_error_degrees = wheel_speeds.sync_error_degrees;
    debug.result = TURN_RESULT_OK;
    publishTurnDebug(debug);
    setSynchronizedEncoderTurnSpeeds(motors, direction_sign, wheel_speeds);
    dly_tsk(etrobo_app::CONTROL_PERIOD_US);
  }

  const EncoderTurnTravel command_stop_travel =
    readEncoderTurnTravel(motors, left_start, right_start);
  debug.encoder_stop_degrees = command_stop_travel.average_degrees;
  coastThenBrakeTurn(motors);
  const EncoderTurnTravel final_travel =
    readEncoderTurnTravel(motors, left_start, right_start);
  debug.active = false;
  debug.current_heading = hub_imu_get_heading();
  debug.encoder_degrees = final_travel.average_degrees;
  debug.left_encoder_degrees = final_travel.left_degrees;
  debug.right_encoder_degrees = final_travel.right_degrees;
  debug.encoder_sync_error_degrees =
    final_travel.left_degrees - final_travel.right_degrees;
  debug.heading_error = direction_sign * robotDegreesForEncoderTurn(
    target_encoder_degrees - debug.encoder_degrees);
  debug.turn_speed_deg_s = 0;
  debug.left_turn_speed_deg_s = 0;
  debug.right_turn_speed_deg_s = 0;
  debug.result = TURN_RESULT_TIMEOUT;
  publishTurnDebug(debug);
  reset_straight_pid_heading();
  return TURN_RESULT_TIMEOUT;
}

int turn_to_heading(int speed, double target_heading)
{
  etrobo_app::DriveMotors motors;
  if (!etrobo_app::getDriveMotors(&motors)) {
    return TURN_RESULT_MOTOR_ERROR;
  }

  const double start_heading = hub_imu_get_heading();
  const double signed_target_degrees =
    angleError(target_heading, start_heading);
  const double target_degrees = std::fabs(signed_target_degrees);
  if (target_degrees <= etrobo_app::GYRO_TOLERANCE_DEG) {
    // 既に格子方位内ならモーターを動かさず、次の直進目標だけを厳密値へ揃える。
    setStraightPidTargetHeading(target_heading);
    return TURN_RESULT_OK;
  }

  // 絶対方位では左右の角度倍率やオフセットを適用しない。
  // それらを足すと、補正後の目標が方位格子そのものから外れるためである。
  const int direction_sign = signFromDouble(signed_target_degrees);
  const int command_degrees =
    static_cast<int>(std::round(signed_target_degrees));
  return runTurnControl(motors,
                        speed,
                        command_degrees,
                        start_heading,
                        target_heading,
                        target_degrees,
                        direction_sign);
}

turn_debug_t turn_get_debug(void)
{
  turn_debug_t debug;

  loc_cpu();
  debug = turn_debug;
  unl_cpu();

  return debug;
}

straight_debug_t straight_get_debug(void)
{
  straight_debug_t debug;

  loc_cpu();
  debug = straight_debug;
  unl_cpu();

  return debug;
}
