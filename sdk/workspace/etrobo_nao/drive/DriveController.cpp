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

struct StraightSpeedProfile {
  int start_speed_deg_s;
  int cruise_speed_deg_s;
  int end_speed_deg_s;
  int accel_degrees;
  int decel_degrees;
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

double averageEncoderTravelDegrees(const etrobo_app::DriveMotors &motors,
                                   int32_t left_start,
                                   int32_t right_start)
{
  const int32_t left_delta = pup_motor_get_count(motors.left) - left_start;
  const int32_t right_delta = pup_motor_get_count(motors.right) - right_start;
  return (std::fabs(static_cast<double>(left_delta)) +
          std::fabs(static_cast<double>(right_delta))) / 2.0;
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
  if (progress < 0.0) {
    progress = 0.0;
  }
  if (progress > 1.0) {
    progress = 1.0;
  }

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

int selectProfileSpeed(const StraightSpeedProfile &profile,
                       double travelled_degrees,
                       int target_degrees)
{
  const double accel_degrees =
    static_cast<double>(profile.accel_degrees);
  const double decel_degrees =
    static_cast<double>(profile.decel_degrees);
  const double profile_degrees = accel_degrees + decel_degrees;

  if (target_degrees <= 0) {
    return 0;
  }

  if (profile_degrees > 0.0 && target_degrees <= profile_degrees) {
    const double accel_ratio =
      accel_degrees > 0.0 ? accel_degrees / profile_degrees : 0.0;
    const double peak_position =
      accel_degrees > 0.0 && decel_degrees > 0.0 ?
      static_cast<double>(target_degrees) * accel_ratio :
      0.0;
    const double peak_progress =
      static_cast<double>(target_degrees) / profile_degrees;
    const int peak_speed =
      interpolateSpeed(profile.start_speed_deg_s,
                       profile.cruise_speed_deg_s,
                       peak_progress);

    if (peak_position > 0.0 && travelled_degrees < peak_position) {
      return interpolateSpeed(profile.start_speed_deg_s,
                              peak_speed,
                              travelled_degrees / peak_position);
    }

    const double decel_position =
      static_cast<double>(target_degrees) - peak_position;
    if (decel_position <= 0.0) {
      return interpolateSpeed(profile.start_speed_deg_s,
                              profile.end_speed_deg_s,
                              travelled_degrees /
                              static_cast<double>(target_degrees));
    }

    const double remaining_degrees =
      static_cast<double>(target_degrees) - travelled_degrees;
    return interpolateSpeed(profile.end_speed_deg_s,
                            peak_speed,
                            remaining_degrees / decel_position);
  }

  if (accel_degrees > 0.0 && travelled_degrees < accel_degrees) {
    return interpolateSpeed(profile.start_speed_deg_s,
                            profile.cruise_speed_deg_s,
                            travelled_degrees / accel_degrees);
  }

  const double remaining_degrees =
    static_cast<double>(target_degrees) - travelled_degrees;
  if (decel_degrees > 0.0 && remaining_degrees <= decel_degrees) {
    return interpolateSpeed(profile.end_speed_deg_s,
                            profile.cruise_speed_deg_s,
                            remaining_degrees / decel_degrees);
  }

  return profile.cruise_speed_deg_s;
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

drive_result_t driveStraightByEncoderProfile(const StraightSpeedProfile &profile,
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

    const int profiled_speed =
      selectProfileSpeed(profile, travelled_degrees, target_degrees);
    const int speed_deg_s =
      direction * applyMinimumStraightSpeed(profiled_speed,
                                            profile.start_speed_deg_s,
                                            profile.end_speed_deg_s);
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

}  // namespace

void stop_motors(void)
{
  etrobo_app::DriveMotors motors;
  if (etrobo_app::getDriveMotors(&motors)) {
    etrobo_app::brakeMotors(motors);
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

drive_result_t drive_straight_profile_mm(int start_speed, int cruise_speed,
                                         int end_speed,
                                         int accel_distance_mm,
                                         int decel_distance_mm,
                                         int distance_mm)
{
  StraightSpeedProfile profile = {};
  profile.start_speed_deg_s = absoluteValue(start_speed);
  profile.cruise_speed_deg_s = absoluteValue(cruise_speed);
  profile.end_speed_deg_s = absoluteValue(end_speed);
  profile.accel_degrees = static_cast<int>(
    etrobo_app::mmToEncoderDegrees(absoluteValue(accel_distance_mm)));
  profile.decel_degrees = static_cast<int>(
    etrobo_app::mmToEncoderDegrees(absoluteValue(decel_distance_mm)));
  const int encoder_degrees =
    static_cast<int>(etrobo_app::mmToEncoderDegrees(distance_mm));
  return driveStraightByEncoderProfile(profile, encoder_degrees, true);
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

  const int base_speed =
    applyMinimumTurnSpeed(
      etrobo_app::clampMotorSpeed(absoluteValue(speed)));
  const double start_heading = hub_imu_get_heading();
  const double target_heading =
    start_heading + turnSign(direction) * target_degrees;
  const int direction_sign = turnSign(direction);
  const int32_t left_start = pup_motor_get_count(motors.left);
  const int32_t right_start = pup_motor_get_count(motors.right);
  const double encoder_limit =
    encoderDegreesForTurn(target_degrees) * etrobo_app::ENCODER_LIMIT_MARGIN +
    etrobo_app::ENCODER_LIMIT_EXTRA_DEG;
  turn_debug_t debug = {};
  debug.active = true;
  debug.phase = 1;
  debug.command_degrees = degrees;
  debug.direction = direction_sign;
  debug.max_speed_deg_s = base_speed;
  debug.result = TURN_RESULT_OK;
  debug.start_heading = start_heading;
  debug.target_degrees = target_degrees;
  debug.target_heading = target_heading;
  debug.current_heading = start_heading;
  debug.heading_error = direction_sign * target_degrees;
  debug.encoder_limit_degrees = encoder_limit;
  publishTurnDebug(debug);

  TurnProgressState turn_progress = {false, 0.0, 0.0};
  int result = runTurnPidUntilStable(motors,
                                     target_degrees,
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
  if (std::fabs(settled_error) <= etrobo_app::GYRO_TOLERANCE_DEG) {
    result = TURN_RESULT_OK;
  }

  for (int attempt = 0;
       attempt < etrobo_app::TURN_SETTLED_CORRECTION_ATTEMPTS &&
       result != TURN_RESULT_MOTOR_ERROR &&
       result != TURN_RESULT_ENCODER_LIMIT &&
       std::fabs(settled_error) > etrobo_app::GYRO_TOLERANCE_DEG;
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
    if (std::fabs(settled_error) <= etrobo_app::GYRO_TOLERANCE_DEG) {
      result = TURN_RESULT_OK;
    }
  }

  if (result == TURN_RESULT_OK &&
      std::fabs(settled_error) > etrobo_app::GYRO_TOLERANCE_DEG) {
    result = TURN_RESULT_TIMEOUT;
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
