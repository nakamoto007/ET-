#include "CompetitionScenario.h"

#include "kernel_cfg.h"
#include "app.h"
#include "BluetoothSender.h"
#include "ColorSensorService.h"
#include "CompetitionSections.h"
#include "DriveController.h"
#include "HubIMUCorrection.h"
#include "RobotController.h"
#include "RobotConfig.h"

#include <cmath>
#include <cstdio>
#include <cstdint>

#include <spike/hub/display.h>
#include <spike/hub/imu.h>

namespace {

competition_scenario_state_t current_state =
  COMPETITION_SCENARIO_STATE_START;
char failure_step = '?';
char failure_reason = '?';

double normalizeHeadingDelta(double delta)
{
  while (delta > 180.0) {
    delta -= 360.0;
  }
  while (delta < -180.0) {
    delta += 360.0;
  }
  return delta;
}

void showImuDriftResult(double drift_degrees)
{
  char text[16];
  const int drift_tenths =
    static_cast<int>(std::round(drift_degrees * 10.0));
  const int abs_tenths =
    drift_tenths < 0 ? -drift_tenths : drift_tenths;
  const char sign = drift_tenths < 0 ? '-' : '+';
  const int length =
    std::snprintf(text, sizeof(text), "D%c%d.%d",
                  sign, abs_tenths / 10, abs_tenths % 10);
  if (length > 0 && static_cast<size_t>(length) < sizeof(text)) {
    hub_display_text_scroll(text, 300);
  }
}

void showHeadingResult(char prefix, double heading_degrees)
{
  char text[16];
  const int heading_tenths =
    static_cast<int>(std::round(heading_degrees * 10.0));
  const int abs_tenths =
    heading_tenths < 0 ? -heading_tenths : heading_tenths;
  const char sign = heading_tenths < 0 ? '-' : '+';
  const int length =
    std::snprintf(text, sizeof(text), "%c%c%d.%d",
                  prefix, sign, abs_tenths / 10, abs_tenths % 10);
  if (length > 0 && static_cast<size_t>(length) < sizeof(text)) {
    hub_display_text_scroll(text, 300);
  }
}

void setScenarioState(competition_scenario_state_t state)
{
  loc_cpu();
  current_state = state;
  unl_cpu();
}

char robotInitFailureReason(robot_init_result_t result)
{
  switch (result) {
  case ROBOT_INIT_DEVICE_ERROR:
    return 'D';
  case ROBOT_INIT_MOTOR_ERROR:
    return 'M';
  case ROBOT_INIT_IMU_ERROR:
  case ROBOT_INIT_IMU_TIMEOUT:
    return 'I';
  case ROBOT_INIT_OK:
    return 'O';
  default:
    return '?';
  }
}

void rememberFailure(char step, char reason)
{
  failure_step = step;
  failure_reason = reason;
  setScenarioState(COMPETITION_SCENARIO_STATE_FAILED);
}

void showFailureLoop(void)
{
  while (1) {
    hub_display_char(failure_step);
    dly_tsk(800 * 1000);
    hub_display_char(failure_reason);
    dly_tsk(800 * 1000);
  }
}

bool showSectionResult(competition_section_result_t result)
{
  if (result.ok) {
    return true;
  }
  rememberFailure(result.step, result.reason);
  return false;
}

void startBackgroundTasks(void)
{
  bluetooth_sender_start();
  (void)act_tsk(BLUETOOTH_CONNECTION_TASK);
  (void)act_tsk(SENSOR_LOG_TASK);
}

bool initializeRobot(void)
{
  hub_display_char('I');
  const robot_init_result_t init_result = initialize_robot();
  if (init_result == ROBOT_INIT_OK) {
    return true;
  }

  rememberFailure('I', robotInitFailureReason(init_result));
  return false;
}

void waitForBluetoothLog(void)
{
  if (!etrobo_app::WAIT_FOR_BLUETOOTH_LOG) {
    return;
  }

  while (!bluetooth_sender_is_ready()) {
    hub_display_char(bluetooth_sender_is_connected() ? 'b' : 'B');
    dly_tsk(etrobo_app::SENSOR_CSV_LOG_INTERVAL_US);
  }
  hub_display_char('A');
}

int32_t sampleReflection(char display_char)
{
  hub_display_char(display_char);
  wait_for_force_start();

  int64_t sum = 0;
  int count = 0;
  for (int sample = 0;
       sample < etrobo_app::COLOR_REFLECTION_CALIBRATION_SAMPLE_COUNT;
       ++sample) {
    (void)color_sensor_service_update();
    color_sensor_values_t values = {};
    if (color_sensor_service_get_values(&values)) {
      sum += values.reflection;
      ++count;
    }
    dly_tsk(etrobo_app::COLOR_REFLECTION_CALIBRATION_SAMPLE_INTERVAL_US);
  }

  if (count <= 0) {
    return -1;
  }
  return static_cast<int32_t>((sum + count / 2) / count);
}

bool calibrateColorReflection(void)
{
  const int32_t white_reflection = sampleReflection('W');
  if (white_reflection < 0) {
    rememberFailure('W', 'C');
    return false;
  }

  const int32_t black_reflection = sampleReflection('K');
  if (black_reflection < 0) {
    rememberFailure('K', 'C');
    return false;
  }

  if (!color_sensor_service_set_normalization_reflection(black_reflection,
                                                         white_reflection)) {
    rememberFailure('N', 'R');
    return false;
  }

  hub_display_char('N');
  dly_tsk(300 * 1000);
  return true;
}

void calibrateRobotPoseAndDrift(void)
{
  setScenarioState(COMPETITION_SCENARIO_STATE_CALIBRATE_POSE);
  hub_display_char('C');
  hub_imu_clear_heading_drift_correction();
  calibrate_robot_pose();
  stop_motors();

  const double start_heading = hub_imu_get_raw_heading();
  dly_tsk(etrobo_app::IMU_DRIFT_CALIBRATION_TIME_US);
  const double end_heading = hub_imu_get_raw_heading();
  const double drift_degrees =
    normalizeHeadingDelta(end_heading - start_heading);
  const double drift_minutes =
    static_cast<double>(etrobo_app::IMU_DRIFT_CALIBRATION_TIME_US) /
    (60.0 * 1000.0 * 1000.0);
  const double drift_deg_per_min =
    drift_minutes > 0.0 ? drift_degrees / drift_minutes : 0.0;

  hub_imu_set_heading_drift_rate(static_cast<float>(drift_deg_per_min));
  reset_straight_pid_heading();
  showImuDriftResult(drift_deg_per_min);
}

bool runBottleColorCheckpoint(void)
{
  if (!showSectionResult(run_line_trace_to_bottle_section())) {
    return false;
  }
  return showSectionResult(run_bottle_color_carry_section());
}

bool runCurrentChallengeLap(void)
{
  waitForBluetoothLog();

  if (!calibrateColorReflection()) {
    return false;
  }

  setScenarioState(COMPETITION_SCENARIO_STATE_WAIT_FOR_START);
  hub_display_char('F');
  wait_for_force_start();

  calibrateRobotPoseAndDrift();

  setScenarioState(COMPETITION_SCENARIO_STATE_WAIT_FOR_START);
  hub_display_char('F');
  wait_for_force_start();

  if (!runBottleColorCheckpoint()) {
    return false;
  }

  setScenarioState(COMPETITION_SCENARIO_STATE_CALIBRATE_POSE);
  hub_display_char('C');
  calibrate_robot_pose();

  if (!showSectionResult(run_challenge_section())) {
    return false;
  }

  showHeadingResult('H', hub_imu_get_heading());

  if (!showSectionResult(run_bottle_push_section())) {
    return false;
  }

  if (!showSectionResult(run_goal_section())) {
    return false;
  }

  setScenarioState(COMPETITION_SCENARIO_STATE_FINISHED);
  hub_display_char('E');
  dly_tsk(50 * 1000);
  return true;
}

bool runChallengeOnlyTestLap(void)
{
  waitForBluetoothLog();

  setScenarioState(COMPETITION_SCENARIO_STATE_WAIT_FOR_START);
  hub_display_char('F');
  wait_for_force_start();

  calibrateRobotPoseAndDrift();

  setScenarioState(COMPETITION_SCENARIO_STATE_WAIT_FOR_START);
  hub_display_char('F');
  wait_for_force_start();

  if (!showSectionResult(run_challenge_section())) {
    return false;
  }

  showHeadingResult('H', hub_imu_get_heading());

  setScenarioState(COMPETITION_SCENARIO_STATE_FINISHED);
  hub_display_char('E');
  dly_tsk(300 * 1000);
  return true;
}

}  // namespace

void competition_scenario_set_state(competition_scenario_state_t state)
{
  setScenarioState(state);
}

void competition_scenario_run(void)
{
  setScenarioState(COMPETITION_SCENARIO_STATE_START);
  startBackgroundTasks();

  if (!initializeRobot()) {
    showFailureLoop();
  }

  waitForBluetoothLog();
  (void)sta_cyc(ROBOT_SENSOR_CYC);

  while (1) {
    if (!runCurrentChallengeLap()) {
      showFailureLoop();
    }
  }
}

void competition_scenario_run_challenge_test(void)
{
  setScenarioState(COMPETITION_SCENARIO_STATE_START);
  startBackgroundTasks();

  if (!initializeRobot()) {
    showFailureLoop();
  }

  waitForBluetoothLog();
  (void)sta_cyc(ROBOT_SENSOR_CYC);

  while (1) {
    if (!runChallengeOnlyTestLap()) {
      showFailureLoop();
    }
  }
}

competition_scenario_state_t competition_scenario_get_state(void)
{
  competition_scenario_state_t state;

  loc_cpu();
  state = current_state;
  unl_cpu();
  return state;
}
