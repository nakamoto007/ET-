#include "kernel_cfg.h"
#include "app.h"
#include "BluetoothSender.h"
#include "CompetitionScenario.h"
#include "DriveController.h"
#include "RobotConfig.h"
#include "RobotStateController.h"
#include "SensorCsvLogger.h"

#include <spike/hub/display.h>

#if defined(__GNUC__)
#define APP_UNUSED __attribute__((unused))
#else
#define APP_UNUSED
#endif

namespace {

char test_failure_step = '?';
char test_failure_reason = '?';
const int TEST_STRAIGHT_START_SPEED = 200;
const int TEST_STRAIGHT_CRUISE_SPEED = 800;
const int TEST_STRAIGHT_ACCEL_DISTANCE_MM = 150;
const int TEST_STRAIGHT_CRUISE_DISTANCE_MM = 200;
const int TEST_STRAIGHT_DECEL_DISTANCE_MM = 150;
const int TEST_TURN_SPEED_DEG_S = 160;

char testDriveFailureReason(drive_result_t result)
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

char testTurnFailureReason(int result)
{
  switch (result) {
  case TURN_RESULT_ENCODER_LIMIT:
    return 'E';
  case TURN_RESULT_TIMEOUT:
    return 'T';
  case TURN_RESULT_MOTOR_ERROR:
    return 'M';
  case TURN_RESULT_OK:
    return 'O';
  default:
    return '?';
  }
}

void rememberTestFailure(char step, char reason)
{
  test_failure_step = step;
  test_failure_reason = reason;
}

APP_UNUSED void showTestFailureLoop(void)
{
  // 失敗した手順と原因を交互に表示して、最後の状態を読み取りやすくする。
  while (1) {
    hub_display_char(test_failure_step);
    dly_tsk(800 * 1000);
    hub_display_char(test_failure_reason);
    dly_tsk(800 * 1000);
  }
}

bool showTestDriveResult(char step, drive_result_t result)
{
  if (result == DRIVE_RESULT_OK) {
    return true;
  }

  rememberTestFailure(step, testDriveFailureReason(result));
  return false;
}

bool showTestTurnResult(char step, int result)
{
  if (result == TURN_RESULT_OK) {
    return true;
  }

  rememberTestFailure(step, testTurnFailureReason(result));
  return false;
}

APP_UNUSED bool runProfiledStraight(void)
{
  if (!showTestDriveResult('A',
                           speed_up(TEST_STRAIGHT_START_SPEED,
                                    TEST_STRAIGHT_CRUISE_SPEED,
                                    TEST_STRAIGHT_ACCEL_DISTANCE_MM))) {
    return false;
  }
  if (!showTestDriveResult('M',
                           drive_straight_mm_keep_speed(
                             TEST_STRAIGHT_CRUISE_SPEED,
                             TEST_STRAIGHT_CRUISE_DISTANCE_MM))) {
    return false;
  }
  if (!showTestDriveResult('D',
                           speed_down(TEST_STRAIGHT_CRUISE_SPEED,
                                      0,
                                      TEST_STRAIGHT_DECEL_DISTANCE_MM))) {
    return false;
  }
  return true;
}

APP_UNUSED bool runRightAngleTurnTest(void)
{
  return showTestTurnResult('R', turn(TEST_TURN_SPEED_DEG_S, 90));
}

}  // namespace

void robot_control_task(intptr_t unused)
{
  (void)unused;
  robot_state_controller_step();
  ext_tsk();
}

void robot_sensor_task(intptr_t unused)
{
  (void)unused;
  robot_sensor_services_step();
  ext_tsk();
}

void sensor_log_task(intptr_t unused)
{
  (void)unused;

  int elapsed_ms = 0;
  bool header_sent = false;

  for (;;) {
    if (!bluetooth_sender_is_ready()) {
      header_sent = false;
    } else {
      if (!header_sent) {
        sensor_csv_logger_print_header();
        header_sent = true;
      }
      sensor_csv_logger_print_row(elapsed_ms);
      elapsed_ms += etrobo_app::SENSOR_CSV_LOG_INTERVAL_US / 1000;
    }
    dly_tsk(etrobo_app::SENSOR_CSV_LOG_INTERVAL_US);
  }
}

void main_task(intptr_t unused)
{
  (void)unused;

  if (etrobo_app::RUN_CHALLENGE_ONLY_TEST) {
    competition_scenario_run_challenge_test();
  } else {
    competition_scenario_run();
  }
  
  ext_tsk();
}
