#include "CompetitionSections.h"

#include "kernel_cfg.h"
#include "app.h"
#include "challenges.h"
#include "CompetitionScenario.h"
#include "DriveController.h"
#include "LineTraceController.h"
#include "RobotConfig.h"
#include "RobotController.h"
#include "RobotStateController.h"
#include "SensorLiftController.h"
#include "../sensors/UltrasonicSensor.h"

#include <spike/hub/display.h>

namespace {

detected_color_t detected_bottle_color = COLOR_DETECT_UNKNOWN;

char sectionDriveFailureReason(drive_result_t result)
{
  switch (result) {
  case DRIVE_RESULT_TIMEOUT:
    return 'T';
  case DRIVE_RESULT_MOTOR_ERROR:
    return 'M';
  case DRIVE_RESULT_OK:
    return 'O';
  default:
    return '?';
  }
}

char sectionLineTraceFailureReason(line_trace_result_t result)
{
  switch (result) {
  case LINE_TRACE_RESULT_TIMEOUT:
    return 'T';
  case LINE_TRACE_RESULT_DEVICE_ERROR:
    return 'D';
  case LINE_TRACE_RESULT_OK:
    return 'O';
  default:
    return '?';
  }
}

char sectionChallengeFailureReason(challenge_run_result_t result)
{
  switch (result) {
  case CHALLENGE_RUN_RESULT_INVALID_STEP:
    return 'S';
  case CHALLENGE_RUN_RESULT_DRIVE_FAILED:
    return 'D';
  case CHALLENGE_RUN_RESULT_TURN_FAILED:
    return 'T';
  case CHALLENGE_RUN_RESULT_EMPTY_STEPS:
    return 'E';
  case CHALLENGE_RUN_RESULT_NULL_STEPS:
    return 'N';
  case CHALLENGE_RUN_RESULT_OK:
    return 'O';
  default:
    return '?';
  }
}

bool wasBottleDetectedByUltrasonic(void)
{
  const ultrasonic_sensor_status_t ultrasonic =
    ultrasonic_sensor_get_status();
  return ultrasonic.enabled && ultrasonic.ready && ultrasonic.obstacle;
}

competition_section_result_t lowerSensorArmWhileDrivingStraight(void)
{
  hub_display_char('D');
  if (!sensor_lift_start_down()) {
    stop_motors();
    return competition_section_fail('D', 'M');
  }

  reset_straight_pid_heading();
  int driven_mm = 0;
  while (sensor_lift_get_status().busy) {
    if (driven_mm >=
        etrobo_app::LINE_TRACE_TO_BOTTLE_ARM_LOWER_MAX_DISTANCE_MM) {
      stop_motors();
      sensor_lift_stop();
      return competition_section_fail('D', 'T');
    }

    const drive_result_t drive_result =
      drive_straight_mm_keep_speed(
        etrobo_app::LINE_TRACE_TO_BOTTLE_ARM_LOWER_SPEED_DEG_S,
        etrobo_app::LINE_TRACE_TO_BOTTLE_ARM_LOWER_STEP_MM);
    if (drive_result != DRIVE_RESULT_OK) {
      stop_motors();
      sensor_lift_stop();
      return competition_section_fail('D',
                                      sectionDriveFailureReason(
                                        drive_result));
    }
    driven_mm += etrobo_app::LINE_TRACE_TO_BOTTLE_ARM_LOWER_STEP_MM;
  }

  const sensor_lift_status_t status = sensor_lift_get_status();
  if (status.state == SENSOR_LIFT_STATE_ERROR) {
    stop_motors();
    return competition_section_fail('D', 'M');
  }

  return competition_section_ok();
}

char colorDisplayChar(detected_color_t color)
{
  switch (color) {
  case COLOR_DETECT_BLACK:
    return 'K';
  case COLOR_DETECT_GRAY:
    return 'A';
  case COLOR_DETECT_WHITE:
    return 'W';
  case COLOR_DETECT_RED:
    return 'R';
  case COLOR_DETECT_BLUE:
    return 'B';
  case COLOR_DETECT_YELLOW:
    return 'Y';
  case COLOR_DETECT_GREEN:
    return 'G';
  case COLOR_DETECT_UNKNOWN:
  default:
    return '?';
  }
}

competition_section_result_t waitForSensorLift(char step)
{
  while (1) {
    const sensor_lift_status_t status = sensor_lift_get_status();
    if (!status.busy) {
      if (status.state == SENSOR_LIFT_STATE_ERROR) {
        return competition_section_fail(step, 'M');
      }
      return competition_section_ok();
    }
    dly_tsk(ROBOT_SENSOR_PERIOD);
  }
}

competition_section_result_t moveSensorArm(char step, bool up)
{
  hub_display_char(step);
  const bool started = up ? sensor_lift_start_up()
                          : sensor_lift_start_down();
  if (!started) {
    return competition_section_fail(step, 'M');
  }
  return waitForSensorLift(step);
}

detected_color_t sampleBottleColor(void)
{
  int counts[COLOR_DETECT_GREEN + 1] = {};

  for (int sample = 0; sample < etrobo_app::BOTTLE_COLOR_SAMPLE_COUNT;
       ++sample) {
    dly_tsk(etrobo_app::BOTTLE_COLOR_SAMPLE_INTERVAL_US);
    const color_detector_status_t status =
      color_detector_get_status();
    if (!status.ready) {
      continue;
    }
    const int color_index = static_cast<int>(status.color);
    if (color_index >= 0 && color_index <= COLOR_DETECT_GREEN) {
      ++counts[color_index];
    }
  }

  detected_color_t best_color = COLOR_DETECT_UNKNOWN;
  int best_count = 0;
  for (int color_index = static_cast<int>(COLOR_DETECT_BLACK);
       color_index <= static_cast<int>(COLOR_DETECT_GREEN);
       ++color_index) {
    if (counts[color_index] > best_count) {
      best_count = counts[color_index];
      best_color = static_cast<detected_color_t>(color_index);
    }
  }
  return best_color;
}

competition_section_result_t detectBottleColor(void)
{
  competition_scenario_set_state(COMPETITION_SCENARIO_STATE_DETECT_COLOR);
  hub_display_char('C');

  detected_bottle_color = sampleBottleColor();
  if (detected_bottle_color == COLOR_DETECT_UNKNOWN) {
    return competition_section_fail('C', 'U');
  }

  hub_display_char(colorDisplayChar(detected_bottle_color));
  dly_tsk(300 * 1000);
  return competition_section_ok();
}

competition_section_result_t approachBottleForColor(void)
{
  hub_display_char('P');
  reset_straight_pid_heading();
  const drive_result_t result =
    drive_straight_mm(etrobo_app::BOTTLE_COLOR_APPROACH_SPEED_DEG_S,
                      etrobo_app::BOTTLE_COLOR_APPROACH_DISTANCE_MM);
  if (result != DRIVE_RESULT_OK) {
    return competition_section_fail('P',
                                    sectionDriveFailureReason(result));
  }
  return competition_section_ok();
}

int requiredBlueZoneCount(detected_color_t bottle_color)
{
  switch (bottle_color) {
  case COLOR_DETECT_YELLOW:
    return 1;
  case COLOR_DETECT_BLUE:
    return 2;
  case COLOR_DETECT_RED:
    return 3;
  default:
    return 0;
  }
}

competition_section_result_t runColorAwareLineTraceToBlack(void)
{
  competition_scenario_set_state(
    COMPETITION_SCENARIO_STATE_LINE_TRACE_AFTER_BOTTLE);
  hub_display_char('L');

  const int required_blue_count = requiredBlueZoneCount(detected_bottle_color);
  if (required_blue_count <= 0) {
    return competition_section_fail('B', 'C');
  }

  const robot_line_trace_options_t options = {
    ROBOT_LINE_TRACE_DISTANCE_UNLIMITED_MM,
    LINE_TRACE_ULTRASONIC_DISABLED_US,
    true,
  };
  if (!robot_state_controller_start_line_trace(&options)) {
    return competition_section_fail('L',
                                    sectionLineTraceFailureReason(
                                      robot_state_controller_get_status()
                                        .line_trace_result));
  }

  int blue_count = 0;
  bool in_blue_zone = false;
  bool reached_black_after_blue = false;

  (void)sta_cyc(ROBOT_CONTROL_CYC);
  for (int cycle = 0;
       cycle < etrobo_app::BOTTLE_CARRY_COLOR_LINE_TRACE_TIMEOUT_CYCLES &&
       robot_state_controller_is_running();
       ++cycle) {
    const color_detector_status_t color_status = color_detector_get_status();
    if (color_status.ready) {
      if (color_status.color == COLOR_DETECT_BLUE) {
        if (!in_blue_zone) {
          ++blue_count;
          in_blue_zone = true;
          hub_display_char('b');
        }
      } else {
        in_blue_zone = false;
      }

      if (blue_count >= required_blue_count &&
          color_status.color == COLOR_DETECT_BLACK) {
        reached_black_after_blue = true;
        robot_state_controller_stop();
        break;
      }
    }

    dly_tsk(etrobo_app::BOTTLE_CARRY_COLOR_POLL_INTERVAL_US);
  }
  (void)stp_cyc(ROBOT_CONTROL_CYC);

  const robot_state_status_t status =
    robot_state_controller_get_status();
  if (status.line_trace_result != LINE_TRACE_RESULT_OK) {
    return competition_section_fail('L',
                                    sectionLineTraceFailureReason(
                                      status.line_trace_result));
  }

  if (!reached_black_after_blue) {
    robot_state_controller_stop();
    return competition_section_fail('B', 'T');
  }
  return competition_section_ok();
}

competition_section_result_t turnByDegrees(char step, int degrees)
{
  hub_display_char(step);
  const int result =
    turn(etrobo_app::CHALLENGE_STEP_TURN_SPEED_DEG_S, degrees);
  if (result != TURN_RESULT_OK) {
    stop_motors();
    return competition_section_fail(step, 'T');
  }
  return competition_section_ok();
}

competition_section_result_t driveUntilColor(char step,
                                             detected_color_t target_color,
                                             int *travelled_mm)
{
  hub_display_char(step);
  reset_straight_pid_heading();
  int driven_mm = 0;
  while (driven_mm <= etrobo_app::BOTTLE_COLOR_ZONE_SEARCH_MAX_DISTANCE_MM) {
    const color_detector_status_t color_status = color_detector_get_status();
    if (color_status.ready && color_status.color == target_color) {
      *travelled_mm = driven_mm;
      return competition_section_ok();
    }

    const drive_result_t drive_result =
      drive_straight_mm_keep_speed(
        etrobo_app::BOTTLE_COLOR_ZONE_SEARCH_SPEED_DEG_S,
        etrobo_app::BOTTLE_COLOR_ZONE_SEARCH_STEP_MM);
    if (drive_result != DRIVE_RESULT_OK) {
      stop_motors();
      return competition_section_fail(step,
                                      sectionDriveFailureReason(
                                        drive_result));
    }
    driven_mm += etrobo_app::BOTTLE_COLOR_ZONE_SEARCH_STEP_MM;
    dly_tsk(etrobo_app::BOTTLE_CARRY_COLOR_POLL_INTERVAL_US);
  }

  stop_motors();
  return competition_section_fail(step, 'C');
}

competition_section_result_t driveBackTravelledDistance(int distance_mm)
{
  hub_display_char('V');
  reset_straight_pid_heading();
  const drive_result_t drive_result =
    drive_straight_mm(etrobo_app::BOTTLE_COLOR_ZONE_RETURN_SPEED_DEG_S,
                      -distance_mm);
  if (drive_result != DRIVE_RESULT_OK) {
    stop_motors();
    return competition_section_fail('V',
                                    sectionDriveFailureReason(
                                      drive_result));
  }
  return competition_section_ok();
}

competition_section_result_t driveUntilBottomBlueThenBlack(void)
{
  hub_display_char('N');
  reset_straight_pid_heading();
  bool passed_blue = false;
  bool in_blue_zone = false;
  int driven_mm = 0;

  while (driven_mm <= etrobo_app::BOTTLE_BOTTOM_BLUE_TO_BLACK_MAX_DISTANCE_MM) {
    const color_detector_status_t color_status = color_detector_get_status();
    if (color_status.ready) {
      if (color_status.color == COLOR_DETECT_BLUE) {
        in_blue_zone = true;
      } else {
        if (in_blue_zone) {
          passed_blue = true;
        }
        in_blue_zone = false;
      }

      if (passed_blue && color_status.color == COLOR_DETECT_BLACK) {
        stop_motors();
        return competition_section_ok();
      }
    }

    const drive_result_t drive_result =
      drive_straight_mm_keep_speed(
        etrobo_app::BOTTLE_COLOR_ZONE_SEARCH_SPEED_DEG_S,
        etrobo_app::BOTTLE_COLOR_ZONE_SEARCH_STEP_MM);
    if (drive_result != DRIVE_RESULT_OK) {
      stop_motors();
      return competition_section_fail('N',
                                      sectionDriveFailureReason(
                                        drive_result));
    }
    driven_mm += etrobo_app::BOTTLE_COLOR_ZONE_SEARCH_STEP_MM;
    dly_tsk(etrobo_app::BOTTLE_CARRY_COLOR_POLL_INTERVAL_US);
  }

  stop_motors();
  return competition_section_fail('N', 'T');
}

bool failed(const competition_section_result_t &result)
{
  return !result.ok;
}

competition_section_result_t driveForward(char step,
                                          int speed_deg_s,
                                          int distance_mm)
{
  if (distance_mm == 0) {
    return competition_section_ok();
  }

  hub_display_char(step);
  reset_straight_pid_heading();
  const drive_result_t result = drive_straight_mm(speed_deg_s, distance_mm);
  if (result != DRIVE_RESULT_OK) {
    stop_motors();
    return competition_section_fail(step,
                                    sectionDriveFailureReason(result));
  }
  return competition_section_ok();
}

void waitForUltrasonicUpdate(void)
{
  for (int count = 0; count < 5; ++count) {
    dly_tsk(ROBOT_SENSOR_PERIOD);
  }
}

}  // namespace

competition_section_result_t run_line_trace_to_bottle_section(void)
{
  competition_scenario_set_state(
    COMPETITION_SCENARIO_STATE_LINE_TRACE_TO_BOTTLE);

  const competition_section_result_t lower_result =
    lowerSensorArmWhileDrivingStraight();
  if (!lower_result.ok) {
    return lower_result;
  }

  hub_display_char('L');

  const robot_line_trace_options_t options = {
    ROBOT_LINE_TRACE_DISTANCE_UNLIMITED_MM,
    etrobo_app::LINE_TRACE_TO_BOTTLE_ULTRASONIC_START_DELAY_US,
    false,
  };
  const line_trace_result_t result =
    robot_state_controller_run_line_trace(&options);
  if (result != LINE_TRACE_RESULT_OK) {
    return competition_section_fail('L',
                                    sectionLineTraceFailureReason(
                                      result));
  }

  if (!wasBottleDetectedByUltrasonic()) {
    return competition_section_fail('U', 'N');
  }

  hub_display_char('U');
  dly_tsk(50 * 1000);
  return competition_section_ok();
}

competition_section_result_t run_bottle_color_carry_section(void)
{
  competition_section_result_t result = approachBottleForColor();
  if (failed(result)) {
    return result;
  }

  result = moveSensorArm('A', true);
  if (failed(result)) {
    return result;
  }

  result = detectBottleColor();
  if (failed(result)) {
    return result;
  }

  competition_scenario_set_state(COMPETITION_SCENARIO_STATE_LOWER_ARM);
  result = moveSensorArm('D', false);
  if (failed(result)) {
    return result;
  }

  result = runColorAwareLineTraceToBlack();
  if (failed(result)) {
    return result;
  }

  competition_scenario_set_state(COMPETITION_SCENARIO_STATE_CALIBRATE_POSE);
  hub_display_char('C');
  calibrate_robot_pose();

  result = turnByDegrees('R', 90);
  if (failed(result)) {
    return result;
  }

  int travelled_to_color_mm = 0;
  result = driveUntilColor('S',
                           detected_bottle_color,
                           &travelled_to_color_mm);
  if (failed(result)) {
    return result;
  }

  result = turnByDegrees('R', 90);
  if (failed(result)) {
    return result;
  }

  result = driveBackTravelledDistance(travelled_to_color_mm);
  if (failed(result)) {
    return result;
  }

  result = turnByDegrees('R', 90);
  if (failed(result)) {
    return result;
  }

  result = driveUntilBottomBlueThenBlack();
  if (failed(result)) {
    return result;
  }

  return turnByDegrees('L', -90);
}

detected_color_t bottle_color_carry_section_get_color(void)
{
  return detected_bottle_color;
}

competition_section_result_t run_challenge_section(void)
{
  competition_scenario_set_state(COMPETITION_SCENARIO_STATE_CHALLENGE_LAPS);
  hub_display_char('H');

  const challenge_run_result_t result = challenges_run_default_steps();
  if (result != CHALLENGE_RUN_RESULT_OK) {
    return competition_section_fail('H',
                                    sectionChallengeFailureReason(
                                      result));
  }
  return competition_section_ok();
}

competition_section_result_t run_bottle_push_section(void)
{
  competition_scenario_set_state(COMPETITION_SCENARIO_STATE_PUSH_BOTTLE);

  competition_section_result_t result =
    driveForward('P',
                 etrobo_app::BOTTLE_PUSH_SPEED_DEG_S,
                 etrobo_app::BOTTLE_PUSH_APPROACH_DISTANCE_MM);
  if (!result.ok) {
    return result;
  }

  ultrasonic_sensor_set_enabled(true);
  waitForUltrasonicUpdate();

  bool saw_bottle = false;
  int driven_mm = 0;
  reset_straight_pid_heading();
  while (driven_mm < etrobo_app::BOTTLE_PUSH_MAX_DISTANCE_MM) {
    const ultrasonic_sensor_status_t ultrasonic =
      ultrasonic_sensor_get_status();
    if (ultrasonic.enabled && ultrasonic.ready) {
      if (ultrasonic.obstacle) {
        saw_bottle = true;
      } else if (saw_bottle) {
        ultrasonic_sensor_set_enabled(false);
        stop_motors();
        return competition_section_ok();
      }
    }

    hub_display_char('O');
    const drive_result_t push_result =
      drive_straight_mm_keep_speed(etrobo_app::BOTTLE_PUSH_SPEED_DEG_S,
                                   etrobo_app::BOTTLE_PUSH_STEP_MM);
    if (push_result != DRIVE_RESULT_OK) {
      ultrasonic_sensor_set_enabled(false);
      stop_motors();
      return competition_section_fail('O',
                                      sectionDriveFailureReason(
                                        push_result));
    }
    driven_mm += etrobo_app::BOTTLE_PUSH_STEP_MM;
  }

  ultrasonic_sensor_set_enabled(false);
  stop_motors();
  return saw_bottle ? competition_section_ok()
                    : competition_section_fail('O', 'N');
}

competition_section_result_t run_goal_section(void)
{
  competition_scenario_set_state(COMPETITION_SCENARIO_STATE_GOAL_RUN);
  hub_display_char('G');

  if (etrobo_app::GOAL_RUN_TURN_DEGREES != 0) {
    const int turn_result =
      turn(etrobo_app::CHALLENGE_STEP_TURN_SPEED_DEG_S,
           etrobo_app::GOAL_RUN_TURN_DEGREES);
    if (turn_result != TURN_RESULT_OK) {
      stop_motors();
      return competition_section_fail('G', 'T');
    }
  }

  if (etrobo_app::GOAL_RUN_DISTANCE_MM != 0) {
    reset_straight_pid_heading();
    const drive_result_t drive_result =
      drive_straight_mm(etrobo_app::GOAL_RUN_SPEED_DEG_S,
                        etrobo_app::GOAL_RUN_DISTANCE_MM);
    if (drive_result != DRIVE_RESULT_OK) {
      stop_motors();
      return competition_section_fail('G',
                                      sectionDriveFailureReason(
                                        drive_result));
    }
  }

  stop_motors();
  return competition_section_ok();
}
