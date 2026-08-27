#include "app.h"
#include "LineTraceController.h"
#include "DriveBase.h"
#include "LineTracer.h"
#include "RobotConfig.h"

#include <spike/pup/motor.h>

namespace {

line_trace_result_t last_line_trace_result = LINE_TRACE_RESULT_OK;

int absoluteValue(int value)
{
  return value < 0 ? -value : value;
}

double absoluteValue(double value)
{
  return value < 0.0 ? -value : value;
}

bool prepareLineTracer(etrobo_app::DriveMotors *motors)
{
  if (!etrobo_app::getDriveMotors(motors)) {
    last_line_trace_result = LINE_TRACE_RESULT_DEVICE_ERROR;
    return false;
  }

  LineTracer_Configure(etrobo_app::LEFT_MOTOR_PORT,
                       etrobo_app::RIGHT_MOTOR_PORT,
                       etrobo_app::COLOR_SENSOR_PORT);
  return true;
}

line_trace_result_t runLineTracerStep(void)
{
  if (!LineTracer_Run()) {
    last_line_trace_result = LINE_TRACE_RESULT_DEVICE_ERROR;
    return last_line_trace_result;
  }

  last_line_trace_result = LINE_TRACE_RESULT_OK;
  return last_line_trace_result;
}

double travelledDistanceMm(const etrobo_app::DriveMotors &motors,
                           double left_start_mm,
                           double right_start_mm)
{
  const double left_mm =
    etrobo_app::encoderDegreesToMm(pup_motor_get_count(motors.left));
  const double right_mm =
    etrobo_app::encoderDegreesToMm(pup_motor_get_count(motors.right));
  return absoluteValue(
    etrobo_app::averageWheelDistanceMm(left_mm - left_start_mm,
                                       right_mm - right_start_mm));
}

void brakeLineTraceMotors(const etrobo_app::DriveMotors *prepared_motors)
{
  if (prepared_motors != nullptr) {
    etrobo_app::brakeMotors(*prepared_motors);
    return;
  }

  etrobo_app::DriveMotors motors;
  if (etrobo_app::getDriveMotors(&motors)) {
    etrobo_app::brakeMotors(motors);
  }
}

}  // namespace

line_trace_result_t line_trace_step(void)
{
  return runLineTracerStep();
}

line_trace_result_t run_line_trace_cycles(int cycles)
{
  etrobo_app::DriveMotors motors;
  if (cycles <= 0) {
    brakeLineTraceMotors(nullptr);
    return LINE_TRACE_RESULT_OK;
  }

  if (!prepareLineTracer(&motors)) {
    return LINE_TRACE_RESULT_DEVICE_ERROR;
  }

  for (int cycle = 0; cycle < cycles; ++cycle) {
    const line_trace_result_t result = runLineTracerStep();
    if (result != LINE_TRACE_RESULT_OK) {
      brakeLineTraceMotors(&motors);
      return result;
    }
    dly_tsk(LINE_TRACER_CONTROL_PERIOD_US);
  }

  brakeLineTraceMotors(&motors);
  return LINE_TRACE_RESULT_OK;
}

line_trace_result_t run_line_trace_mm(int distance_mm)
{
  etrobo_app::DriveMotors motors;
  if (!prepareLineTracer(&motors)) {
    return LINE_TRACE_RESULT_DEVICE_ERROR;
  }

  if (distance_mm == 0) {
    brakeLineTraceMotors(&motors);
    return LINE_TRACE_RESULT_OK;
  }

  const double target_distance_mm =
    static_cast<double>(absoluteValue(distance_mm));
  const double left_start_mm =
    etrobo_app::encoderDegreesToMm(pup_motor_get_count(motors.left));
  const double right_start_mm =
    etrobo_app::encoderDegreesToMm(pup_motor_get_count(motors.right));

  for (int cycle = 0; cycle < etrobo_app::MAX_DRIVE_CYCLES; ++cycle) {
    if (travelledDistanceMm(motors, left_start_mm, right_start_mm) >=
        target_distance_mm) {
      brakeLineTraceMotors(&motors);
      return LINE_TRACE_RESULT_OK;
    }

    const line_trace_result_t result = runLineTracerStep();
    if (result != LINE_TRACE_RESULT_OK) {
      brakeLineTraceMotors(&motors);
      return result;
    }
    dly_tsk(LINE_TRACER_CONTROL_PERIOD_US);
  }

  brakeLineTraceMotors(&motors);
  return LINE_TRACE_RESULT_TIMEOUT;
}

void stop_line_trace(void)
{
  brakeLineTraceMotors(nullptr);
}
