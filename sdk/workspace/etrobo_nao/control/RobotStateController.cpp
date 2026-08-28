#include "RobotStateController.h"
#include "kernel_cfg.h"
#include "app.h"
#include "ColorDetector.h"
#include "ColorSensorService.h"
#include "DriveBase.h"
#include "LineTracer.h"
#include "RobotConfig.h"
#include "SensorLiftController.h"
#include "../sensors/UltrasonicSensor.h"

#include <kernel.h>
#include <spike/pup/motor.h>

namespace {

struct RobotStateController {
  robot_state_status_t status;
  etrobo_app::DriveMotors drive_motors;
  double left_start_mm;
  double right_start_mm;
  int cycles;
  int distance_mm;
  bool use_distance_limit;
  int ultrasonic_start_delay_us;
  bool ultrasonic_started;
  bool update_color_during_line_trace;
};

RobotStateController controller = {};

double absoluteValue(double value)
{
  return value < 0.0 ? -value : value;
}

double travelledDistanceMm(void)
{
  const double left_mm =
    etrobo_app::encoderDegreesToMm(
      pup_motor_get_count(controller.drive_motors.left));
  const double right_mm =
    etrobo_app::encoderDegreesToMm(
      pup_motor_get_count(controller.drive_motors.right));
  return absoluteValue(
    etrobo_app::averageWheelDistanceMm(left_mm - controller.left_start_mm,
                                       right_mm - controller.right_start_mm));
}

void setStopped(line_trace_result_t result)
{
  color_sensor_service_set_reflection_only(false);
  stop_line_trace();
  controller.status.state =
    result == LINE_TRACE_RESULT_OK ? ROBOT_RUN_STATE_STOPPED
                                   : ROBOT_RUN_STATE_ERROR;
  controller.status.running = false;
  controller.status.line_trace_result = result;
}

bool shouldStopForDistance(void)
{
  if (!controller.use_distance_limit) {
    return false;
  }
  return travelledDistanceMm() >=
         static_cast<double>(controller.distance_mm);
}

bool shouldStopForObstacle(void)
{
  if (!etrobo_app::ENABLE_ULTRASONIC_STOP) {
    return false;
  }
  const ultrasonic_sensor_status_t ultrasonic =
    ultrasonic_sensor_get_status();
  return ultrasonic.enabled && ultrasonic.ready && ultrasonic.obstacle;
}

void updateUltrasonicActivation(void)
{
  if (controller.ultrasonic_start_delay_us <=
      LINE_TRACE_ULTRASONIC_DISABLED_US) {
    return;
  }

  if (!etrobo_app::ENABLE_ULTRASONIC_SENSOR || controller.ultrasonic_started) {
    return;
  }

  const int elapsed_us = controller.cycles * ROBOT_CONTROL_PERIOD;
  if (elapsed_us < controller.ultrasonic_start_delay_us) {
    return;
  }

  ultrasonic_sensor_set_enabled(true);
  controller.ultrasonic_started = true;
}

robot_line_trace_options_t defaultLineTraceOptions(void)
{
  robot_line_trace_options_t options = {};
  options.distance_mm = ROBOT_LINE_TRACE_DISTANCE_UNLIMITED_MM;
  options.ultrasonic_start_delay_us = LINE_TRACE_ULTRASONIC_DISABLED_US;
  options.update_color_during_line_trace = false;
  return options;
}

robot_line_trace_options_t resolveLineTraceOptions(
  const robot_line_trace_options_t *options)
{
  if (options == nullptr) {
    return defaultLineTraceOptions();
  }
  return *options;
}

}  // namespace

void robot_state_controller_reset(void)
{
  color_sensor_service_set_reflection_only(false);
  ultrasonic_sensor_set_enabled(false);
  controller = {};
  controller.status.state = ROBOT_RUN_STATE_IDLE;
  controller.status.line_trace_result = LINE_TRACE_RESULT_OK;
  sensor_lift_reset();
}

bool robot_state_controller_start_line_trace(
  const robot_line_trace_options_t *options)
{
  robot_state_controller_reset();
  const robot_line_trace_options_t resolved_options =
    resolveLineTraceOptions(options);

  if (!etrobo_app::getDriveMotors(&controller.drive_motors)) {
    controller.status.state = ROBOT_RUN_STATE_ERROR;
    controller.status.line_trace_result = LINE_TRACE_RESULT_DEVICE_ERROR;
    return false;
  }

  color_sensor_service_set_reflection_only(
    !resolved_options.update_color_during_line_trace);
  LineTracer_Configure(etrobo_app::LEFT_MOTOR_PORT,
                       etrobo_app::RIGHT_MOTOR_PORT,
                       etrobo_app::COLOR_SENSOR_PORT);

  if (resolved_options.distance_mm == 0) {
    color_sensor_service_set_reflection_only(false);
    etrobo_app::brakeMotors(controller.drive_motors);
    controller.status.state = ROBOT_RUN_STATE_STOPPED;
    controller.status.line_trace_result = LINE_TRACE_RESULT_OK;
    return true;
  }

  controller.distance_mm = resolved_options.distance_mm;
  controller.use_distance_limit = resolved_options.distance_mm > 0;
  controller.left_start_mm =
    etrobo_app::encoderDegreesToMm(
      pup_motor_get_count(controller.drive_motors.left));
  controller.right_start_mm =
    etrobo_app::encoderDegreesToMm(
      pup_motor_get_count(controller.drive_motors.right));
  controller.cycles = 0;
  controller.ultrasonic_start_delay_us =
    resolved_options.ultrasonic_start_delay_us;
  controller.ultrasonic_started = false;
  controller.update_color_during_line_trace =
    resolved_options.update_color_during_line_trace;
  controller.status.state = ROBOT_RUN_STATE_LINE_TRACE;
  controller.status.running = true;
  controller.status.line_trace_result = LINE_TRACE_RESULT_OK;
  return true;
}

line_trace_result_t robot_state_controller_run_line_trace(
  const robot_line_trace_options_t *options)
{
  if (!robot_state_controller_start_line_trace(options)) {
    sensor_lift_stop();
    return controller.status.line_trace_result;
  }

  if (!controller.status.running) {
    sensor_lift_stop();
    return controller.status.line_trace_result;
  }

  (void)sta_cyc(ROBOT_CONTROL_CYC);

  while (controller.status.running) {
    dly_tsk(100 * 1000);
  }

  (void)stp_cyc(ROBOT_CONTROL_CYC);
  sensor_lift_stop();
  return controller.status.line_trace_result;
}

void robot_state_controller_step(void)
{
  if (controller.status.state != ROBOT_RUN_STATE_LINE_TRACE) {
    return;
  }

  if (controller.use_distance_limit &&
      controller.cycles >= etrobo_app::MAX_DRIVE_CYCLES) {
    setStopped(LINE_TRACE_RESULT_TIMEOUT);
    return;
  }

  updateUltrasonicActivation();

  if (shouldStopForDistance() || shouldStopForObstacle()) {
    setStopped(LINE_TRACE_RESULT_OK);
    return;
  }

  const line_trace_result_t trace_result =
    line_trace_step();
  if (trace_result != LINE_TRACE_RESULT_OK) {
    setStopped(trace_result);
    return;
  }

  ++controller.cycles;
}

void robot_sensor_services_step(void)
{
  if (controller.status.state != ROBOT_RUN_STATE_LINE_TRACE ||
      controller.update_color_during_line_trace) {
    (void)color_sensor_service_update();
  }
  color_detector_step();
  ultrasonic_sensor_step();
  sensor_lift_step();
}

void robot_state_controller_stop(void)
{
  setStopped(LINE_TRACE_RESULT_OK);
  sensor_lift_stop();
}

bool robot_state_controller_is_running(void)
{
  return controller.status.running;
}

robot_state_status_t robot_state_controller_get_status(void)
{
  return controller.status;
}
