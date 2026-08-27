#include "app.h"
#include "SensorCsvLogger.h"
#include "AllSensors.h"
#include "BluetoothSender.h"
#include "ColorDetector.h"
#include "DriveController.h"
#include "LineTraceController.h"
#include "LineTracer.h"
#include "RobotConfig.h"

#include <stdio.h>

namespace {

int boolToInt(bool value)
{
  return value ? 1 : 0;
}

uint32_t last_turn_debug_update_count = 0;
uint32_t last_straight_debug_update_count = 0;

void sensorCsvLoggerPrintStraightRow(int elapsed_ms)
{
  const straight_debug_t straight_debug = straight_get_debug();
  if (!straight_debug.active &&
      straight_debug.update_count == last_straight_debug_update_count) {
    return;
  }
  last_straight_debug_update_count = straight_debug.update_count;

  char line[256];
  const int length = snprintf(
      line, sizeof(line),
      "straight,%d,%lu,%d,%d,%d,%d,%d,%d,%.2f,%.2f,%.2f,%.1f,%.1f,%.1f,%.1f\n",
      elapsed_ms,
      static_cast<unsigned long>(straight_debug.update_count),
      boolToInt(straight_debug.active),
      straight_debug.cycle,
      straight_debug.base_speed_deg_s,
      straight_debug.left_speed_deg_s,
      straight_debug.right_speed_deg_s,
      straight_debug.result,
      straight_debug.target_heading,
      straight_debug.current_heading,
      straight_debug.heading_error,
      straight_debug.correction_deg_s,
      straight_debug.correction_limit_deg_s,
      straight_debug.travelled_degrees,
      straight_debug.target_degrees);
  if (length > 0 && static_cast<size_t>(length) < sizeof(line)) {
    (void)bluetooth_sender_send(line);
  }
}

void sensorCsvLoggerPrintTurnRow(int elapsed_ms)
{
  const turn_debug_t turn_debug = turn_get_debug();
  if (!turn_debug.active &&
      turn_debug.update_count == last_turn_debug_update_count) {
    return;
  }
  last_turn_debug_update_count = turn_debug.update_count;

  char line[256];
  const int length = snprintf(
      line, sizeof(line),
      "turn,%d,%lu,%d,%d,%d,%d,%d,%.2f,%.2f,%.2f,%.2f,%d,%d,%d,%.1f,%.1f\n",
      elapsed_ms,
      static_cast<unsigned long>(turn_debug.update_count),
      boolToInt(turn_debug.active),
      turn_debug.phase,
      turn_debug.command_degrees,
      turn_debug.direction,
      turn_debug.max_speed_deg_s,
      turn_debug.start_heading,
      turn_debug.target_heading,
      turn_debug.current_heading,
      turn_debug.heading_error,
      turn_debug.turn_speed_deg_s,
      turn_debug.stable_count,
      turn_debug.result,
      turn_debug.encoder_degrees,
      turn_debug.encoder_limit_degrees);
  if (length > 0 && static_cast<size_t>(length) < sizeof(line)) {
    (void)bluetooth_sender_send(line);
  }
}

}  // namespace

void sensor_csv_logger_print_header(void)
{
  // BluetoothSenderの256byte制限に収まるCSVヘッダー。
  (void)bluetooth_sender_send(
          "ms,dok,lc,rc,ls,rs,lp,rp,"
          "cok,ref,nref,r,g,b,h,s,v,v8,det,"
          "lt_ref,lt_nref,lt_err,lt_der,lt_base,lt_line,lt_imu,lt_turn,lt_lp,lt_rp,edge,lt_mode,lt_curve,lt_entry,"
          "fok,fn,imu,imucal,ax,ay,az,gz,hd\n");
  (void)bluetooth_sender_send(
          "kind,ms,tseq,tact,tphase,tcmd,tdir,tmax,tstart,ttgt,thd,terr,tspd,tst,tres,tenc,telim\n");
  (void)bluetooth_sender_send(
          "kind,ms,sseq,sact,scyc,sbase,sleft,sright,sres,stgt,shd,serr,scorr,slim,senc,stenc\n");
}

void sensor_csv_logger_print_row(int elapsed_ms)
{
  const all_sensor_values_t sensors = get_all_sensor_values();
  const color_detector_status_t color_detection = color_detector_get_status();
  const line_tracer_debug_t line_trace = LineTracer_GetDebug();
  char line[256];
  const int length = snprintf(
      line, sizeof(line),
      "%d,%d,%ld,%ld,%ld,%ld,%ld,%ld,"
      "%d,%ld,%ld,%u,%u,%u,%u,%u,%u,%u,%d,"
      "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,"
      "%d,%.3f,%d,%d,%.3f,%.3f,%.3f,%.3f,%.3f\n",
      elapsed_ms,
      boolToInt(sensors.drive_motors.drive_motors_ready),
      static_cast<long>(sensors.drive_motors.left_motor_count),
      static_cast<long>(sensors.drive_motors.right_motor_count),
      static_cast<long>(sensors.drive_motors.left_motor_speed),
      static_cast<long>(sensors.drive_motors.right_motor_speed),
      static_cast<long>(sensors.drive_motors.left_motor_power),
      static_cast<long>(sensors.drive_motors.right_motor_power),
      boolToInt(sensors.color.color_sensor_ready),
      static_cast<long>(sensors.color.reflection),
      static_cast<long>(sensors.color.normalized_reflection),
      static_cast<unsigned int>(sensors.color.rgb_r),
      static_cast<unsigned int>(sensors.color.rgb_g),
      static_cast<unsigned int>(sensors.color.rgb_b),
      static_cast<unsigned int>(sensors.color.hsv_h),
      static_cast<unsigned int>(sensors.color.hsv_s),
      static_cast<unsigned int>(sensors.color.hsv_v),
      static_cast<unsigned int>(sensors.color.hsv_v8),
      color_detection.ready ?
        static_cast<int>(color_detection.color) :
        static_cast<int>(COLOR_DETECT_UNKNOWN),
      line_trace.reflection,
      line_trace.normalized_reflection,
      line_trace.error,
      line_trace.derivative,
      line_trace.base_power,
      line_trace.line_turn_power,
      line_trace.imu_turn_power,
      line_trace.turn_power,
      line_trace.left_power,
      line_trace.right_power,
      line_trace.edge,
      line_trace.path_state,
      line_trace.curve_level,
      line_trace.curve_entry,
      boolToInt(sensors.force.force_sensor_ready),
      sensors.force.force_n,
      boolToInt(sensors.imu.imu_ready),
      sensors.imu.calibration_status,
      sensors.imu.acceleration_x,
      sensors.imu.acceleration_y,
      sensors.imu.acceleration_z,
      sensors.imu.angular_velocity_z,
      sensors.imu.heading);
  if (length > 0 && static_cast<size_t>(length) < sizeof(line)) {
    (void)bluetooth_sender_send(line);
  }
  sensorCsvLoggerPrintStraightRow(elapsed_ms);
  sensorCsvLoggerPrintTurnRow(elapsed_ms);
}

void sensor_csv_logger_flush(void)
{
  // BluetoothSenderは呼び出しごとに即時送信する。
}

void run_sensor_csv_logger_seconds(int seconds)
{
  if (seconds <= 0) {
    return;
  }

  const int interval_ms = etrobo_app::SENSOR_CSV_LOG_INTERVAL_US / 1000;
  sensor_csv_logger_print_header();
  for (int elapsed_ms = 0; elapsed_ms <= seconds * 1000;
       elapsed_ms += interval_ms) {
    sensor_csv_logger_print_row(elapsed_ms);
    sensor_csv_logger_flush();
    if (elapsed_ms + interval_ms <= seconds * 1000) {
      dly_tsk(etrobo_app::SENSOR_CSV_LOG_INTERVAL_US);
    }
  }
}
