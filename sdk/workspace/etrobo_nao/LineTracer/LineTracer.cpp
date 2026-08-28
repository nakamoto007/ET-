#include "app.h"
#include "LineTracer.h"
#include "ColorSensorService.h"
#include <stdio.h>

#include <spike/hub/imu.h>
#include <spike/pup/motor.h>
#include <spike/pup/colorsensor.h>

/* 関数プロトタイプ宣言 */
static int16_t steering_amount_calculation(void);
static void motor_drive_control(int);
static int clamp_motor_power(int);
static int current_base_power(void);
static int detect_curve_level(int);
static int calculate_line_turn_power(float, float, float, int);
static int adjust_turn_power(int);
static int adjust_detection_turn_power(int);
static int select_line_turn_power(int, int);
static float read_imu_yaw_rate(void);
static float filter_straight_imu_yaw_rate(float);
static bool should_apply_straight_imu_correction(int);
static int calculate_straight_imu_correction(int, float);
static int apply_straight_imu_correction(int, int);

static pup_motor_t *fg_left_motor;
static pup_motor_t *fg_right_motor;
static pup_device_t *fg_color_sensor;
static bool fg_configured = false;
static float previous_error = 0.0F;
static bool has_previous_error = false;
static float filtered_error = 0.0F;
static bool has_filtered_error = false;
static float filtered_imu_yaw_rate = 0.0F;
static bool has_filtered_imu_yaw_rate = false;
static int curve_detect_count = 0;
static int straight_detect_count = 0;
static int curve_detect_sign = 0;
static int path_state = LINE_TRACER_PATH_UNKNOWN;
static int curve_level = LINE_TRACER_CURVE_LEVEL_NONE;
static int curve_entry_latched = 0;
static line_tracer_debug_t fg_debug = {};

static int absolute_int(int value)
{
    if (value < 0) {
        return -value;
    }
    return value;
}

static float absolute_float(float value)
{
    if (value < 0.0F) {
        return -value;
    }
    return value;
}

static void reset_trace_detection(void)
{
    previous_error = 0.0F;
    has_previous_error = false;
    filtered_error = 0.0F;
    has_filtered_error = false;
    filtered_imu_yaw_rate = 0.0F;
    has_filtered_imu_yaw_rate = false;
    curve_detect_count = 0;
    straight_detect_count = 0;
    curve_detect_sign = 0;
    path_state = LINE_TRACER_PATH_UNKNOWN;
    curve_level = LINE_TRACER_CURVE_LEVEL_NONE;
    curve_entry_latched = 0;

    loc_cpu();
    fg_debug = {};
    fg_debug.path_state = LINE_TRACER_PATH_UNKNOWN;
    fg_debug.curve_level = LINE_TRACER_CURVE_LEVEL_NONE;
    fg_debug.base_power = LINE_TRACER_STRAIGHT_BASE_POWER;
    unl_cpu();
}

static void update_path_detection(int curve_turn_power,
                                  int straight_turn_power,
                                  float derivative,
                                  float imu_yaw_rate)
{
    int next_curve_entry = 0;
    const int next_curve_level = detect_curve_level(curve_turn_power);
    const int abs_curve_steering = absolute_int(curve_turn_power);
    const int abs_straight_steering = absolute_int(straight_turn_power);
    const int next_curve_sign =
        curve_turn_power > 0 ? 1 : (curve_turn_power < 0 ? -1 : 0);
    const float abs_derivative = absolute_float(derivative);
    const float abs_yaw_rate = absolute_float(imu_yaw_rate);
    const bool imu_curve_like =
        LINE_TRACER_IMU_PATH_DETECTION_ENABLE &&
        abs_yaw_rate >= LINE_TRACER_IMU_CURVE_YAW_RATE_MIN &&
        abs_curve_steering >= LINE_TRACER_IMU_CURVE_STEERING_MIN;
    const bool imu_straight_like =
        !LINE_TRACER_IMU_PATH_DETECTION_ENABLE ||
        path_state == LINE_TRACER_PATH_STRAIGHT ||
        abs_yaw_rate <= LINE_TRACER_IMU_STRAIGHT_YAW_RATE_MAX;
    const bool curve_like =
        abs_curve_steering >= LINE_TRACER_CURVE_STEERING_MIN ||
        abs_derivative >= LINE_TRACER_CURVE_DERIVATIVE_MIN ||
        imu_curve_like;
    const bool straight_like =
        abs_straight_steering <= LINE_TRACER_STRAIGHT_STEERING_MAX &&
        abs_derivative <= LINE_TRACER_STRAIGHT_DERIVATIVE_MAX &&
        imu_straight_like;

    if (curve_like && next_curve_sign != 0) {
        if (curve_detect_sign == next_curve_sign) {
            ++curve_detect_count;
        } else {
            curve_detect_sign = next_curve_sign;
            curve_detect_count = 1;
        }
        straight_detect_count = 0;
        if (curve_detect_count >= LINE_TRACER_CURVE_DETECT_COUNT) {
            if (path_state != LINE_TRACER_PATH_CURVE) {
                next_curve_entry = 1;
            }
            path_state = LINE_TRACER_PATH_CURVE;
            curve_level = next_curve_level;
        }
    } else if (straight_like) {
        ++straight_detect_count;
        curve_detect_count = 0;
        curve_detect_sign = 0;
        if (straight_detect_count >= LINE_TRACER_STRAIGHT_DETECT_COUNT) {
            path_state = LINE_TRACER_PATH_STRAIGHT;
            curve_level = LINE_TRACER_CURVE_LEVEL_NONE;
        }
    } else {
        curve_detect_count = 0;
        straight_detect_count = 0;
        curve_detect_sign = 0;
    }

    if (next_curve_entry != 0) {
        curve_entry_latched = 1;
    }
}

void LineTracer_Configure(pbio_port_id_t left_motor_port, pbio_port_id_t right_motor_port, pbio_port_id_t color_sensor_port)
{
  fg_configured = false;
  fg_color_sensor = NULL;
  fg_left_motor = NULL;
  fg_right_motor = NULL;

  /* センサー入力ポートの設定 */
  if (color_sensor_service_lock()) {
    fg_color_sensor = pup_color_sensor_get_device(color_sensor_port);
    if (fg_color_sensor != NULL) {
      pup_color_sensor_light_on(fg_color_sensor);
    }
    color_sensor_service_unlock();
  }
  fg_left_motor   = pup_motor_get_device(left_motor_port);
  fg_right_motor   = pup_motor_get_device(right_motor_port);

  reset_trace_detection();
  if (fg_color_sensor == NULL || fg_left_motor == NULL ||
      fg_right_motor == NULL) {
    return;
  }

  pup_motor_setup(fg_left_motor,PUP_DIRECTION_COUNTERCLOCKWISE,true);
  pup_motor_setup(fg_right_motor,PUP_DIRECTION_CLOCKWISE,true);

  fg_configured = true;

}


/* ライントレースタスク(3msec周期で関数コールされる) */
bool LineTracer_Run(void)
{
    int turn_power; /* ステアリング補正power */

    if (!fg_configured || fg_left_motor == NULL ||
        fg_right_motor == NULL || fg_color_sensor == NULL) {
        return false;
    }
    /* ステアリング補正powerの計算 */
    turn_power = steering_amount_calculation();

    /* 走行モータ制御 */
    motor_drive_control(turn_power);

    return true;
}

void tracer_task(intptr_t unused) {

    (void)unused;
    (void)LineTracer_Run();

    /* タスク終了 */
    ext_tsk();
}

/* ステアリング操舵量の計算 */
static int16_t steering_amount_calculation(void){

    uint16_t  target_brightness; /* 正規化後の目標輝度値 */
    float     error;             /* 目標輝度との差分値 */
    float     control_error;     /* 平滑化とデッドバンド後の制御用差分値 */
    float     derivative;        /* 目標輝度との差分の変化量 */
    float     raw_imu_yaw_rate;  /* カーブ判定用のIMU Z軸角速度 */
    float     filtered_imu_yaw_rate_for_correction; /* 直線補正用の平滑化角速度 */
    int       straight_line_turn_power; /* 直線用の弱いライン補正power */
    int       curve_line_turn_power; /* カーブ用のライン補正power */
    int       adjusted_curve_line_turn_power; /* 実際に使うカーブ補正power */
    int       detection_turn_power; /* カーブ判定用のライン補正power */
    int       line_turn_power;   /* ラインセンサーだけで作った補正power */
    int       imu_turn_power;    /* 直進中だけ足すIMU補正power */
    int       turn_power;        /* 左右モーターへ足し引きする補正power */
    int32_t   ref;
    int32_t   normalized_ref;

    /* 正規化後の中央値をライン境界の目標にする。 */
    target_brightness = TARGET_BRIGHTNESS;

    /* カラーセンサ値の取得 */
    if (fg_color_sensor != NULL && color_sensor_service_lock()) {
        ref = pup_color_sensor_reflection(fg_color_sensor);
        color_sensor_service_unlock();
    } else {
        ref = 0;
    }
    color_sensor_service_store_reflection(ref);
    normalized_ref = color_sensor_service_normalize_reflection(ref);

    /* 目標輝度値と正規化したカラーセンサ値の差分を計算 */
    error = (float)(target_brightness - normalized_ref);
    if (has_filtered_error) {
        filtered_error +=
            LINE_TRACER_ERROR_FILTER_ALPHA * (error - filtered_error);
    } else {
        filtered_error = error;
        has_filtered_error = true;
    }
    control_error = filtered_error;
    if (absolute_float(control_error) <= LINE_TRACER_ERROR_DEADBAND) {
        control_error = 0.0F;
    }

    derivative = 0.0F;
    if (has_previous_error) {
        derivative = (control_error - previous_error) / CONTROL_DT_SEC;
    } else {
        has_previous_error = true;
    }
    raw_imu_yaw_rate = read_imu_yaw_rate();
    filtered_imu_yaw_rate_for_correction =
        filter_straight_imu_yaw_rate(raw_imu_yaw_rate);

    straight_line_turn_power =
        calculate_line_turn_power(control_error,
                                  derivative,
                                  LINE_TRACER_STRAIGHT_KP,
                                  LINE_TRACER_STRAIGHT_STEERING_DEADBAND);
    curve_line_turn_power =
        calculate_line_turn_power(control_error,
                                  derivative,
                                  LINE_TRACER_CURVE_KP,
                                  LINE_TRACER_CURVE_STEERING_DEADBAND);
    adjusted_curve_line_turn_power = adjust_turn_power(curve_line_turn_power);
    detection_turn_power =
        adjust_detection_turn_power(
            calculate_line_turn_power(control_error,
                                      derivative,
                                      LINE_TRACER_CURVE_DETECT_KP,
                                      LINE_TRACER_CURVE_STEERING_DEADBAND));
    previous_error = control_error;
    update_path_detection(detection_turn_power,
                          straight_line_turn_power,
                          derivative,
                          raw_imu_yaw_rate);
    line_turn_power =
        select_line_turn_power(straight_line_turn_power,
                               adjusted_curve_line_turn_power);
    imu_turn_power =
        calculate_straight_imu_correction(
            line_turn_power,
            filtered_imu_yaw_rate_for_correction);
    turn_power = apply_straight_imu_correction(line_turn_power, imu_turn_power);

    loc_cpu();
    fg_debug.reflection = (int)ref;
    fg_debug.normalized_reflection = (int)normalized_ref;
    fg_debug.error = (int)control_error;
    fg_debug.derivative = (int)derivative;
    fg_debug.base_power = current_base_power();
    fg_debug.line_turn_power = line_turn_power;
    fg_debug.imu_turn_power = imu_turn_power;
    fg_debug.turn_power = turn_power;
    fg_debug.edge = TRACE_EDGE;
    fg_debug.path_state = path_state;
    fg_debug.curve_level = curve_level;
    fg_debug.curve_entry = curve_entry_latched;
    unl_cpu();

    return (int16_t)turn_power;
}

/* 走行モータ制御 */
static void motor_drive_control(int turn_power){

    int left_motor_power, right_motor_power; /*左右モータ設定パワー*/
    const int base_power = current_base_power();

    /* 左右モータ駆動パワーの計算 */
    left_motor_power  = base_power + turn_power;
    right_motor_power = base_power - turn_power;
    left_motor_power = clamp_motor_power(left_motor_power);
    right_motor_power = clamp_motor_power(right_motor_power);

    loc_cpu();
    fg_debug.left_power = left_motor_power;
    fg_debug.right_power = right_motor_power;
    unl_cpu();

    /* 左右モータ駆動パワーの設定 */
    pup_motor_set_power(fg_left_motor, left_motor_power);
    pup_motor_set_power(fg_right_motor, right_motor_power);

    return;
}

line_tracer_debug_t LineTracer_GetDebug(void)
{
    line_tracer_debug_t debug;

    loc_cpu();
    debug = fg_debug;
    curve_entry_latched = 0;
    fg_debug.curve_entry = 0;
    unl_cpu();

    return debug;
}

static int clamp_motor_power(int power)
{
    if (power > MOTOR_POWER_MAX) {
        return MOTOR_POWER_MAX;
    }
    if (power < MOTOR_POWER_MIN) {
        return MOTOR_POWER_MIN;
    }
    return power;
}

static int current_base_power(void)
{
    if (curve_level == LINE_TRACER_CURVE_LEVEL_NONE) {
        return LINE_TRACER_STRAIGHT_BASE_POWER;
    }
    if (curve_level == LINE_TRACER_CURVE_LEVEL_SHARP) {
        return LINE_TRACER_SHARP_CURVE_BASE_POWER;
    }
    if (curve_level == LINE_TRACER_CURVE_LEVEL_NORMAL) {
        return LINE_TRACER_NORMAL_CURVE_BASE_POWER;
    }
    if (curve_level == LINE_TRACER_CURVE_LEVEL_GENTLE) {
        return LINE_TRACER_GENTLE_CURVE_BASE_POWER;
    }
    return LINE_TRACER_STRAIGHT_BASE_POWER;
}

static int detect_curve_level(int turn_power)
{
    const int abs_steering = absolute_int(turn_power);
    if (abs_steering >= LINE_TRACER_SHARP_CURVE_STEERING_MIN) {
        return LINE_TRACER_CURVE_LEVEL_SHARP;
    }
    if (abs_steering >= LINE_TRACER_NORMAL_CURVE_STEERING_MIN) {
        return LINE_TRACER_CURVE_LEVEL_NORMAL;
    }
    if (abs_steering >= LINE_TRACER_CURVE_STEERING_MIN) {
        return LINE_TRACER_CURVE_LEVEL_GENTLE;
    }
    return LINE_TRACER_CURVE_LEVEL_GENTLE;
}

static int calculate_line_turn_power(float control_error,
                                     float derivative,
                                     float kp,
                                     int steering_deadband)
{
    int16_t steering_amount =
        (int16_t)(kp * control_error + STEERING_KD * derivative);

    if (absolute_int((int)steering_amount) <= steering_deadband) {
        steering_amount = 0;
    }

    return clamp_motor_power((int)steering_amount * TRACE_EDGE);
}

static int adjust_turn_power(int turn_power)
{
    if (absolute_int(turn_power) >= LINE_TRACER_CURVE_TURN_BOOST_MIN) {
        turn_power =
            (int)((float)turn_power * LINE_TRACER_CURVE_TURN_GAIN);
    }
    if (turn_power < 0) {
        turn_power =
            (int)((float)turn_power * LINE_TRACER_LEFT_CURVE_TURN_GAIN);
    } else if (turn_power > 0) {
        turn_power =
            (int)((float)turn_power * LINE_TRACER_RIGHT_CURVE_TURN_GAIN);
    }
    return clamp_motor_power(turn_power);
}

static int adjust_detection_turn_power(int turn_power)
{
    if (absolute_int(turn_power) >=
        LINE_TRACER_CURVE_DETECT_TURN_BOOST_MIN) {
        turn_power =
            (int)((float)turn_power *
                  LINE_TRACER_CURVE_DETECT_TURN_GAIN);
    }
    return clamp_motor_power(turn_power);
}

static int select_line_turn_power(int raw_turn_power, int curve_turn_power)
{
    if (path_state == LINE_TRACER_PATH_CURVE ||
        curve_level != LINE_TRACER_CURVE_LEVEL_NONE) {
        return curve_turn_power;
    }
    return raw_turn_power;
}

static float read_imu_yaw_rate(void)
{
    float angular_velocity[3] = {0.0F, 0.0F, 0.0F};

    hub_imu_get_angular_velocity(angular_velocity);
    return angular_velocity[2];
}

static float filter_straight_imu_yaw_rate(float raw_imu_yaw_rate)
{
    if (has_filtered_imu_yaw_rate) {
        filtered_imu_yaw_rate +=
            LINE_TRACER_STRAIGHT_IMU_YAW_RATE_FILTER_ALPHA *
            (raw_imu_yaw_rate - filtered_imu_yaw_rate);
    } else {
        filtered_imu_yaw_rate = raw_imu_yaw_rate;
        has_filtered_imu_yaw_rate = true;
    }
    return filtered_imu_yaw_rate;
}

static bool should_apply_straight_imu_correction(int line_turn_power)
{
    if (!LINE_TRACER_STRAIGHT_IMU_CORRECTION_ENABLE) {
        return false;
    }
    if (path_state != LINE_TRACER_PATH_STRAIGHT) {
        return false;
    }
    return absolute_int(line_turn_power) <=
           LINE_TRACER_STRAIGHT_IMU_LINE_TURN_MAX;
}

static int calculate_straight_imu_correction(int line_turn_power,
                                             float imu_yaw_rate)
{
    int imu_turn_power;

    if (!should_apply_straight_imu_correction(line_turn_power)) {
        return 0;
    }

    const float raw_imu_turn_power =
        -imu_yaw_rate * LINE_TRACER_STRAIGHT_IMU_YAW_RATE_GAIN;
    if (raw_imu_turn_power >= 0.0F) {
        imu_turn_power = (int)(raw_imu_turn_power + 0.5F);
    } else {
        imu_turn_power = (int)(raw_imu_turn_power - 0.5F);
    }
    if (imu_turn_power > LINE_TRACER_STRAIGHT_IMU_CORRECTION_LIMIT) {
        return LINE_TRACER_STRAIGHT_IMU_CORRECTION_LIMIT;
    }
    if (imu_turn_power < -LINE_TRACER_STRAIGHT_IMU_CORRECTION_LIMIT) {
        return -LINE_TRACER_STRAIGHT_IMU_CORRECTION_LIMIT;
    }
    return imu_turn_power;
}

static int apply_straight_imu_correction(int line_turn_power,
                                         int imu_turn_power)
{
    return clamp_motor_power(line_turn_power + imu_turn_power);
}
