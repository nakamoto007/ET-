#include "BottleDetection.h"

#include "app.h"
#include "DriveBase.h"
#include "DriveController.h"
#include "RobotConfig.h"

#include <cmath>
#include <cstdint>

#include <spike/hub/display.h>
#include <spike/hub/imu.h>
#include <spike/pup/motor.h>
#include <spike/pup/ultrasonicsensor.h>

namespace {

struct Pose {
  double x_mm;
  double y_mm;
  int32_t previous_left_count;
  int32_t previous_right_count;
};

struct HeadingPidState {
  double integral;
  double previous_error;
  bool has_previous_error;
};

Pose pose = {};
bottle_detection_status_t status = {};

double degreesToRadians(double degrees)
{
  return degrees * etrobo_app::PI / 180.0;
}

double normalizeHeadingError(double error)
{
  while (error > 180.0) {
    error -= 360.0;
  }
  while (error < -180.0) {
    error += 360.0;
  }
  return error;
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

int roundToInt(double value)
{
  return value >= 0.0 ? static_cast<int>(value + 0.5) :
                        static_cast<int>(value - 0.5);
}

double distanceBetween(double x1, double y1, double x2, double y2)
{
  const double dx = x2 - x1;
  const double dy = y2 - y1;
  return std::sqrt(dx * dx + dy * dy);
}

void publishStatus(void)
{
  const double heading = hub_imu_get_heading();

  loc_cpu();
  status.ready = true;
  status.pose_x_mm = pose.x_mm;
  status.pose_y_mm = pose.y_mm;
  status.heading_deg = heading;
  unl_cpu();
}

void publishObjectStatus(double object_x,
                         double object_y,
                         int32_t distance,
                         double heading)
{
  loc_cpu();
  status.object_found = true;
  status.object_x_mm = object_x;
  status.object_y_mm = object_y;
  status.object_distance_mm = distance;
  status.object_heading_deg = heading;
  unl_cpu();
}

void resetOdometry(const etrobo_app::DriveMotors &motors)
{
  etrobo_app::resetDriveMotorCounts(motors);
  pose.x_mm = 0.0;
  pose.y_mm = 0.0;
  pose.previous_left_count = 0;
  pose.previous_right_count = 0;
  loc_cpu();
  status = {};
  unl_cpu();
  publishStatus();
}

double updateOdometry(const etrobo_app::DriveMotors &motors)
{
  const int32_t left_count = pup_motor_get_count(motors.left);
  const int32_t right_count = pup_motor_get_count(motors.right);
  const double left_distance =
    etrobo_app::encoderDegreesToMm(left_count - pose.previous_left_count);
  const double right_distance =
    etrobo_app::encoderDegreesToMm(right_count - pose.previous_right_count);
  pose.previous_left_count = left_count;
  pose.previous_right_count = right_count;

  const double center_distance =
    etrobo_app::averageWheelDistanceMm(left_distance, right_distance);
  const double heading_radians = degreesToRadians(hub_imu_get_heading());
  pose.x_mm += center_distance * std::cos(heading_radians);
  pose.y_mm += center_distance * std::sin(heading_radians);
  publishStatus();
  return center_distance;
}

void brakeAndSettle(const etrobo_app::DriveMotors &motors)
{
  etrobo_app::brakeMotors(motors);
  dly_tsk(etrobo_app::BOTTLE_DETECTION_SENSOR_SETTLE_TIME_US);
}

int scanSpeedForError(double error_degrees)
{
  const double absolute_error = std::fabs(error_degrees);
  if (absolute_error > 10.0) {
    return etrobo_app::BOTTLE_DETECTION_SCAN_MAX_SPEED_DEG_S;
  }
  if (absolute_error > 3.0) {
    return etrobo_app::BOTTLE_DETECTION_SCAN_MIDDLE_SPEED_DEG_S;
  }
  return etrobo_app::BOTTLE_DETECTION_SCAN_MIN_SPEED_DEG_S;
}

bool scanRightToHeading(const etrobo_app::DriveMotors &motors,
                        double target_heading)
{
  for (int cycle = 0;
       cycle < etrobo_app::BOTTLE_DETECTION_SCAN_STEP_TIMEOUT_CYCLES;
       ++cycle) {
    updateOdometry(motors);

    const double error =
      normalizeHeadingError(target_heading - hub_imu_get_heading());
    if (error <= etrobo_app::BOTTLE_DETECTION_TURN_TOLERANCE_DEG) {
      brakeAndSettle(motors);
      return true;
    }

    const int speed = scanSpeedForError(error);
    etrobo_app::setMotorSpeeds(motors, speed, -speed);
    dly_tsk(etrobo_app::CONTROL_PERIOD_US);
  }

  brakeAndSettle(motors);
  return false;
}

void sortSamples(int32_t *values, int count)
{
  for (int i = 1; i < count; ++i) {
    const int32_t value = values[i];
    int j = i - 1;
    while (j >= 0 && values[j] > value) {
      values[j + 1] = values[j];
      --j;
    }
    values[j + 1] = value;
  }
}

int32_t medianDistance(pup_device_t *ultrasonic_sensor)
{
  int32_t samples[etrobo_app::BOTTLE_DETECTION_DISTANCE_SAMPLE_COUNT] = {};
  int valid_count = 0;

  for (int attempt = 0;
       attempt < etrobo_app::BOTTLE_DETECTION_DISTANCE_MAX_ATTEMPTS &&
       valid_count < etrobo_app::BOTTLE_DETECTION_DISTANCE_SAMPLE_COUNT;
       ++attempt) {
    const int32_t distance =
      pup_ultrasonic_sensor_distance(ultrasonic_sensor);
    if (distance > 0) {
      samples[valid_count++] = distance;
    }
    dly_tsk(etrobo_app::BOTTLE_DETECTION_SAMPLE_INTERVAL_US);
  }

  if (valid_count == 0) {
    return -1;
  }

  while (valid_count < etrobo_app::BOTTLE_DETECTION_DISTANCE_SAMPLE_COUNT) {
    samples[valid_count] = samples[valid_count - 1];
    ++valid_count;
  }

  sortSamples(samples, etrobo_app::BOTTLE_DETECTION_DISTANCE_SAMPLE_COUNT);
  return samples[etrobo_app::BOTTLE_DETECTION_DISTANCE_SAMPLE_COUNT / 2];
}

double headingTo(double target_x, double target_y)
{
  return std::atan2(target_y - pose.y_mm, target_x - pose.x_mm) *
         180.0 / etrobo_app::PI;
}

turn_result_t turnToAbsoluteHeading(double target_heading)
{
  const double error =
    normalizeHeadingError(target_heading - hub_imu_get_heading());
  if (std::fabs(error) <= etrobo_app::BOTTLE_DETECTION_TURN_TOLERANCE_DEG) {
    reset_straight_pid_heading();
    return TURN_RESULT_OK;
  }

  return static_cast<turn_result_t>(
    turn(etrobo_app::BOTTLE_DETECTION_TURN_SPEED_DEG_S, roundToInt(error)));
}

double updateHeadingPid(HeadingPidState *pid, double error)
{
  pid->integral += error * etrobo_app::CONTROL_PERIOD_SEC;
  pid->integral =
    clampDouble(pid->integral,
                etrobo_app::BOTTLE_DETECTION_NAV_INTEGRAL_LIMIT_DEG_SEC);

  double derivative = 0.0;
  if (pid->has_previous_error) {
    derivative =
      (error - pid->previous_error) / etrobo_app::CONTROL_PERIOD_SEC;
  } else {
    pid->has_previous_error = true;
  }
  pid->previous_error = error;

  const double correction =
    etrobo_app::BOTTLE_DETECTION_NAV_KP * error +
    etrobo_app::BOTTLE_DETECTION_NAV_KI * pid->integral +
    etrobo_app::BOTTLE_DETECTION_NAV_KD * derivative;
  return clampDouble(correction,
                     etrobo_app::BOTTLE_DETECTION_NAV_CORRECTION_LIMIT_DEG_S);
}

bottle_detection_result_t driveToCoordinate(
  const etrobo_app::DriveMotors &motors,
  double target_x,
  double target_y)
{
  HeadingPidState heading_pid = {0.0, 0.0, false};

  for (int cycle = 0;
       cycle < etrobo_app::BOTTLE_DETECTION_NAVIGATION_TIMEOUT_CYCLES;
       ++cycle) {
    updateOdometry(motors);

    if (distanceBetween(pose.x_mm, pose.y_mm, target_x, target_y) <=
        etrobo_app::BOTTLE_DETECTION_GOAL_TOLERANCE_MM) {
      brakeAndSettle(motors);
      reset_straight_pid_heading();
      return BOTTLE_DETECTION_RESULT_OK;
    }

    const double target_heading = headingTo(target_x, target_y);
    const double raw_error =
      normalizeHeadingError(target_heading - hub_imu_get_heading());
    const double error =
      std::fabs(raw_error) <= etrobo_app::STRAIGHT_PID_DEADBAND_DEG ?
      0.0 : raw_error;
    const double correction =
      clampDouble(updateHeadingPid(&heading_pid, error),
                  etrobo_app::BOTTLE_DETECTION_DRIVE_SPEED_DEG_S);

    etrobo_app::setMotorSpeeds(
      motors,
      etrobo_app::BOTTLE_DETECTION_DRIVE_SPEED_DEG_S + correction,
      etrobo_app::BOTTLE_DETECTION_DRIVE_SPEED_DEG_S - correction);
    dly_tsk(etrobo_app::CONTROL_PERIOD_US);
  }

  brakeAndSettle(motors);
  reset_straight_pid_heading();
  return BOTTLE_DETECTION_RESULT_DRIVE_TIMEOUT;
}

}  // namespace

bottle_detection_result_t bottle_detection_run(void)
{
  etrobo_app::DriveMotors motors;
  pup_device_t *ultrasonic_sensor =
    pup_ultrasonic_sensor_get_device(etrobo_app::ULTRASONIC_SENSOR_PORT);
  if (!etrobo_app::getDriveMotors(&motors) ||
      ultrasonic_sensor == nullptr ||
      !hub_imu_is_ready()) {
    return BOTTLE_DETECTION_RESULT_DEVICE_ERROR;
  }

  (void)pup_ultrasonic_sensor_light_off(ultrasonic_sensor);
  resetOdometry(motors);

  hub_display_char('L');
  if (turnToAbsoluteHeading(
        etrobo_app::BOTTLE_DETECTION_SCAN_START_DEG) != TURN_RESULT_OK) {
    (void)pup_ultrasonic_sensor_light_off(ultrasonic_sensor);
    return BOTTLE_DETECTION_RESULT_TURN_FAILED;
  }
  updateOdometry(motors);

  double object_x = 0.0;
  double object_y = 0.0;
  bool found = false;

  hub_display_char('B');
  for (int angle = etrobo_app::BOTTLE_DETECTION_SCAN_START_DEG;
       angle <= etrobo_app::BOTTLE_DETECTION_SCAN_END_DEG;
       angle += etrobo_app::BOTTLE_DETECTION_SCAN_STEP_DEG) {
    if (!scanRightToHeading(motors, static_cast<double>(angle))) {
      (void)pup_ultrasonic_sensor_light_off(ultrasonic_sensor);
      return BOTTLE_DETECTION_RESULT_TURN_FAILED;
    }

    const int32_t distance = medianDistance(ultrasonic_sensor);
    const double measured_heading = hub_imu_get_heading();
    updateOdometry(motors);

    if (distance > 0 &&
        distance <= etrobo_app::BOTTLE_DETECTION_MAX_DISTANCE_MM) {
      const double heading_radians = degreesToRadians(measured_heading);
      object_x = pose.x_mm + distance * std::cos(heading_radians);
      object_y = pose.y_mm + distance * std::sin(heading_radians);
      publishObjectStatus(object_x, object_y, distance, measured_heading);
      found = true;
      break;
    }
  }

  brakeAndSettle(motors);
  if (!found) {
    reset_straight_pid_heading();
    (void)pup_ultrasonic_sensor_light_off(ultrasonic_sensor);
    return BOTTLE_DETECTION_RESULT_NOT_FOUND;
  }

  (void)pup_ultrasonic_sensor_light_on(ultrasonic_sensor);

  hub_display_char('T');
  const double object_heading = headingTo(object_x, object_y);
  if (turnToAbsoluteHeading(object_heading) != TURN_RESULT_OK) {
    (void)pup_ultrasonic_sensor_light_off(ultrasonic_sensor);
    return BOTTLE_DETECTION_RESULT_TURN_FAILED;
  }
  updateOdometry(motors);

  const double object_distance =
    distanceBetween(pose.x_mm, pose.y_mm, object_x, object_y);
  if (object_distance <= 1.0) {
    (void)pup_ultrasonic_sensor_light_off(ultrasonic_sensor);
    reset_straight_pid_heading();
    return BOTTLE_DETECTION_RESULT_OK;
  }

  const double unit_x = (object_x - pose.x_mm) / object_distance;
  const double unit_y = (object_y - pose.y_mm) / object_distance;
  const double target_x =
    object_x + etrobo_app::BOTTLE_DETECTION_COLLISION_MARGIN_MM * unit_x;
  const double target_y =
    object_y + etrobo_app::BOTTLE_DETECTION_COLLISION_MARGIN_MM * unit_y;

  hub_display_char('G');
  const bottle_detection_result_t result =
    driveToCoordinate(motors, target_x, target_y);
  if (result != BOTTLE_DETECTION_RESULT_OK) {
    (void)pup_ultrasonic_sensor_light_off(ultrasonic_sensor);
  }
  return result;
}

bottle_detection_result_t bottle_detection_navigate_to_coordinate_mm(
  double target_x_mm,
  double target_y_mm)
{
  etrobo_app::DriveMotors motors;
  if (!etrobo_app::getDriveMotors(&motors) || !hub_imu_is_ready()) {
    return BOTTLE_DETECTION_RESULT_DEVICE_ERROR;
  }

  updateOdometry(motors);

  hub_display_char('N');
  const double target_heading = headingTo(target_x_mm, target_y_mm);
  if (turnToAbsoluteHeading(target_heading) != TURN_RESULT_OK) {
    return BOTTLE_DETECTION_RESULT_TURN_FAILED;
  }
  updateOdometry(motors);

  hub_display_char('D');
  return driveToCoordinate(motors, target_x_mm, target_y_mm);
}

bottle_detection_status_t bottle_detection_get_status(void)
{
  bottle_detection_status_t copied_status;

  loc_cpu();
  copied_status = status;
  unl_cpu();

  return copied_status;
}
