#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include "BottleConfig.h"
#include "ChallengeConfig.h"
#include "ControlConfig.h"
#include "HardwareConfig.h"
#include "LineTraceConfig.h"
#include "LogConfig.h"
#include "SensorConfig.h"

#include <spike/pup/motor.h>

namespace etrobo_app {

struct SensorValueRange {
  int min;
  int max;
};

struct ColorSensorRange {
  SensorValueRange reflection;
  SensorValueRange rgb_r;
  SensorValueRange rgb_g;
  SensorValueRange rgb_b;
  SensorValueRange hsv_h;
  SensorValueRange hsv_s;
  SensorValueRange hsv_v;
  SensorValueRange hsv_v8;
};

struct ColorKindRanges {
  SensorValueRange black;
  SensorValueRange green;
  SensorValueRange yellow;
  SensorValueRange red;
  SensorValueRange blue;
  SensorValueRange white;
};

struct ColorSensorCalibrationRanges {
  ColorKindRanges reflection;
  ColorKindRanges rgb_r;
  ColorKindRanges rgb_g;
  ColorKindRanges rgb_b;
  ColorKindRanges hsv_h;
  ColorKindRanges hsv_s;
  ColorKindRanges hsv_v;
  ColorKindRanges hsv_v8;
};

const SensorValueRange COLOR_SENSOR_REFLECTION_FULL_RANGE = {
  COLOR_SENSOR_REFLECTION_MIN,
  COLOR_SENSOR_REFLECTION_MAX
};
const SensorValueRange COLOR_SENSOR_RGB_FULL_RANGE = {
  COLOR_SENSOR_RGB_RAW_MIN,
  COLOR_SENSOR_RGB_RAW_MAX
};
const SensorValueRange COLOR_SENSOR_HUE_FULL_RANGE = {
  COLOR_SENSOR_HUE_MIN_DEGREES,
  COLOR_SENSOR_HUE_MAX_DEGREES
};
const SensorValueRange COLOR_SENSOR_HSV_PERCENT_FULL_RANGE = {
  COLOR_SENSOR_HSV_PERCENT_MIN,
  COLOR_SENSOR_HSV_PERCENT_MAX
};
const SensorValueRange COLOR_SENSOR_HSV_VALUE8_FULL_RANGE = {
  COLOR_SENSOR_HSV_VALUE8_MIN,
  COLOR_SENSOR_HSV_VALUE8_MAX
};

// 試走会CSVから拾った色別レンジ。データ種類ごとに縦に見る。
const ColorSensorCalibrationRanges COLOR_SENSOR_CALIBRATION_RANGES = {
  // reflection: カラーセンサーの反射光値。
  {
    { 0, 6 },    // 黒。43000msログの反射値2を含む
    { 16, 21 },  // 緑
    { 55, 70 },  // 黄
    { 29, 31 },  // 赤
    { 20, 30 },  // 青
    { 81, 99 }   // 白
  },
  // rgb_r: RGB生値の赤成分。
  {
    { 22, 50 },   // 黒。43000msログのrgb_r=22を下限にする
    { 98, 130 },  // 緑
    { 661, 845 }, // 黄
    { 609, 648 }, // 赤
    { 70, 80 },   // 青
    { 1000, 1020 } // 白
  },
  // rgb_g: RGB生値の緑成分。
  {
    { 28, 55 },   // 黒。43000msログのrgb_g=28を下限にする
    { 233, 306 },  // 緑
    { 632, 815 },  // 黄
    { 117, 128 },  // 赤
    { 250, 300 },  // 青
    { 1000, 1020 } // 白
  },
  // rgb_b: RGB生値の青成分。
  {
    { 28, 65 },   // 黒。43000msログのrgb_b=28を下限にする
    { 172, 229 },  // 緑
    { 398, 499 },  // 黄
    { 181, 195 },  // 赤
    { 500, 550 },  // 青
    { 1000, 1020 } // 白
  },
  // hsv_h: RGBから計算した色相。単位は度。
  {
    { 180, 220 }, // 黒。43000msログのhsv_h=180を含む
    { 150, 158 }, // 緑
    { 53, 55 },   // 黄
    { 353, 353 }, // 赤
    { 200, 230 }, // 青
    { 180, 204 }  // 白: 彩度が低いためHは参考値。白判定には基本使わない
  },
  // hsv_s: RGBから計算した彩度。0-100の百分率。
  {
    { 20, 25 }, // 黒。43000msログのhsv_s=21を含む
    { 55, 59 }, // 緑
    { 40, 41 }, // 黄
    { 80, 81 }, // 赤
    { 80, 90 }, // 青
    { 0, 10 }   // 白
  },
  // hsv_v: RGBから計算した明度。0-100の百分率。
  {
    { 3, 6 },   // 黒。43000msログのhsv_v=3を下限にする
    { 23, 30 }, // 緑
    { 65, 83 }, // 黄
    { 60, 63 }, // 赤
    { 50, 55 }, // 青
    { 90, 100 } // 白
  },
  // hsv_v8: RGBから計算した明度。0-255に換算した値。
  {
    { 7, 16 },    // 黒。43000msログのhsv_v8=7を下限にする
    { 58, 76 },   // 緑
    { 165, 211 }, // 黄
    { 152, 162 }, // 赤
    { 130, 140 }, // 青
    { 233, 254 }  // 白
  }
};

// 色判定で使いやすいよう、データ種類別レンジから色ごとに取り出した名前。
const ColorSensorRange COLOR_BLACK_RANGE = {
  COLOR_SENSOR_CALIBRATION_RANGES.reflection.black,
  COLOR_SENSOR_CALIBRATION_RANGES.rgb_r.black,
  COLOR_SENSOR_CALIBRATION_RANGES.rgb_g.black,
  COLOR_SENSOR_CALIBRATION_RANGES.rgb_b.black,
  COLOR_SENSOR_CALIBRATION_RANGES.hsv_h.black,
  COLOR_SENSOR_CALIBRATION_RANGES.hsv_s.black,
  COLOR_SENSOR_CALIBRATION_RANGES.hsv_v.black,
  COLOR_SENSOR_CALIBRATION_RANGES.hsv_v8.black
};

const ColorSensorRange COLOR_GREEN_RANGE = {
  COLOR_SENSOR_CALIBRATION_RANGES.reflection.green,
  COLOR_SENSOR_CALIBRATION_RANGES.rgb_r.green,
  COLOR_SENSOR_CALIBRATION_RANGES.rgb_g.green,
  COLOR_SENSOR_CALIBRATION_RANGES.rgb_b.green,
  COLOR_SENSOR_CALIBRATION_RANGES.hsv_h.green,
  COLOR_SENSOR_CALIBRATION_RANGES.hsv_s.green,
  COLOR_SENSOR_CALIBRATION_RANGES.hsv_v.green,
  COLOR_SENSOR_CALIBRATION_RANGES.hsv_v8.green
};

const ColorSensorRange COLOR_YELLOW_RANGE = {
  COLOR_SENSOR_CALIBRATION_RANGES.reflection.yellow,
  COLOR_SENSOR_CALIBRATION_RANGES.rgb_r.yellow,
  COLOR_SENSOR_CALIBRATION_RANGES.rgb_g.yellow,
  COLOR_SENSOR_CALIBRATION_RANGES.rgb_b.yellow,
  COLOR_SENSOR_CALIBRATION_RANGES.hsv_h.yellow,
  COLOR_SENSOR_CALIBRATION_RANGES.hsv_s.yellow,
  COLOR_SENSOR_CALIBRATION_RANGES.hsv_v.yellow,
  COLOR_SENSOR_CALIBRATION_RANGES.hsv_v8.yellow
};

const ColorSensorRange COLOR_RED_RANGE = {
  COLOR_SENSOR_CALIBRATION_RANGES.reflection.red,
  COLOR_SENSOR_CALIBRATION_RANGES.rgb_r.red,
  COLOR_SENSOR_CALIBRATION_RANGES.rgb_g.red,
  COLOR_SENSOR_CALIBRATION_RANGES.rgb_b.red,
  COLOR_SENSOR_CALIBRATION_RANGES.hsv_h.red,
  COLOR_SENSOR_CALIBRATION_RANGES.hsv_s.red,
  COLOR_SENSOR_CALIBRATION_RANGES.hsv_v.red,
  COLOR_SENSOR_CALIBRATION_RANGES.hsv_v8.red
};

const ColorSensorRange COLOR_BLUE_RANGE = {
  COLOR_SENSOR_CALIBRATION_RANGES.reflection.blue,
  COLOR_SENSOR_CALIBRATION_RANGES.rgb_r.blue,
  COLOR_SENSOR_CALIBRATION_RANGES.rgb_g.blue,
  COLOR_SENSOR_CALIBRATION_RANGES.rgb_b.blue,
  COLOR_SENSOR_CALIBRATION_RANGES.hsv_h.blue,
  COLOR_SENSOR_CALIBRATION_RANGES.hsv_s.blue,
  COLOR_SENSOR_CALIBRATION_RANGES.hsv_v.blue,
  COLOR_SENSOR_CALIBRATION_RANGES.hsv_v8.blue
};

const ColorSensorRange COLOR_WHITE_RANGE = {
  COLOR_SENSOR_CALIBRATION_RANGES.reflection.white,
  COLOR_SENSOR_CALIBRATION_RANGES.rgb_r.white,
  COLOR_SENSOR_CALIBRATION_RANGES.rgb_g.white,
  COLOR_SENSOR_CALIBRATION_RANGES.rgb_b.white,
  COLOR_SENSOR_CALIBRATION_RANGES.hsv_h.white,
  COLOR_SENSOR_CALIBRATION_RANGES.hsv_s.white,
  COLOR_SENSOR_CALIBRATION_RANGES.hsv_v.white,
  COLOR_SENSOR_CALIBRATION_RANGES.hsv_v8.white
};

// 灰色マーク判定用レンジ。現状は反射光、彩度、明度だけを使う。
const ColorSensorRange GRAY_MARK_RANGE = {
  { 38, 40 },
  COLOR_SENSOR_RGB_FULL_RANGE,
  COLOR_SENSOR_RGB_FULL_RANGE,
  COLOR_SENSOR_RGB_FULL_RANGE,
  COLOR_SENSOR_HUE_FULL_RANGE,
  { 0, 25 },
  { 20, 40 },
  COLOR_SENSOR_HSV_VALUE8_FULL_RANGE
};

// 灰色マークとみなす反射値の下限。
const int GRAY_MARK_REFLECTION_MIN = GRAY_MARK_RANGE.reflection.min;
// 灰色マークとみなす反射値の上限。
const int GRAY_MARK_REFLECTION_MAX = GRAY_MARK_RANGE.reflection.max;
// 灰色マークとみなすHSV彩度Sの上限。灰色は色味が少ないので低くする。
const int GRAY_MARK_MAX_SATURATION = GRAY_MARK_RANGE.hsv_s.max;
// 灰色マークとみなすHSV明度Vの下限。黒の誤検知を防ぐ。
const int GRAY_MARK_MIN_VALUE = GRAY_MARK_RANGE.hsv_v.min;
// 灰色マークとみなすHSV明度Vの上限。白の誤検知を防ぐ。
const int GRAY_MARK_MAX_VALUE = GRAY_MARK_RANGE.hsv_v.max;

// 赤マークとみなすRGBのR下限。試走会CSVの赤レンジを使う。
const int RED_MARK_R_MIN = COLOR_RED_RANGE.rgb_r.min;
// 赤マークとみなすRGBのR上限。
const int RED_MARK_R_MAX = COLOR_RED_RANGE.rgb_r.max;
// 赤マークとみなすRGBのG下限。
const int RED_MARK_G_MIN = COLOR_RED_RANGE.rgb_g.min;
// 赤マークとみなすRGBのG上限。
const int RED_MARK_G_MAX = COLOR_RED_RANGE.rgb_g.max;
// 赤マークとみなすRGBのB下限。
const int RED_MARK_B_MIN = COLOR_RED_RANGE.rgb_b.min;
// 赤マークとみなすRGBのB上限。
const int RED_MARK_B_MAX = COLOR_RED_RANGE.rgb_b.max;
// RがG/Bよりこの値以上大きければ赤らしいとみなす。
const int RED_MARK_RGB_DIFFERENCE_MIN = 25;

// 青マークとみなすRGBのR下限。
const int BLUE_MARK_R_MIN = COLOR_BLUE_RANGE.rgb_r.min;
// 青マークとみなすRGBのR上限。
const int BLUE_MARK_R_MAX = COLOR_BLUE_RANGE.rgb_r.max;
// 青マークとみなすRGBのG下限。
const int BLUE_MARK_G_MIN = COLOR_BLUE_RANGE.rgb_g.min;
// 青マークとみなすRGBのG上限。
const int BLUE_MARK_G_MAX = COLOR_BLUE_RANGE.rgb_g.max;
// 青マークとみなすRGBのB下限。
const int BLUE_MARK_B_MIN = COLOR_BLUE_RANGE.rgb_b.min;
// 青マークとみなすRGBのB上限。
const int BLUE_MARK_B_MAX = COLOR_BLUE_RANGE.rgb_b.max;
// BがR/Gよりこの値以上大きければ青らしいとみなす。
const int BLUE_MARK_RGB_DIFFERENCE_MIN = 25;

// 黄色マークとみなすRGBのR下限。
const int YELLOW_MARK_R_MIN = COLOR_YELLOW_RANGE.rgb_r.min;
// 黄色マークとみなすRGBのR上限。
const int YELLOW_MARK_R_MAX = COLOR_YELLOW_RANGE.rgb_r.max;
// 黄色マークとみなすRGBのG下限。
const int YELLOW_MARK_G_MIN = COLOR_YELLOW_RANGE.rgb_g.min;
// 黄色マークとみなすRGBのG上限。
const int YELLOW_MARK_G_MAX = COLOR_YELLOW_RANGE.rgb_g.max;
// 黄色マークとみなすRGBのB下限。
const int YELLOW_MARK_B_MIN = COLOR_YELLOW_RANGE.rgb_b.min;
// 黄色マークとみなすRGBのB上限。
const int YELLOW_MARK_B_MAX = COLOR_YELLOW_RANGE.rgb_b.max;
// R/GがBよりこの値以上大きければ黄色らしいとみなす。
const int YELLOW_MARK_RGB_DIFFERENCE_MIN = 25;
// 黄色はRとGが近いので、RとGの差をこの値以下にする。
const int YELLOW_MARK_RGB_BALANCE_MAX = 250;

// 白マークとみなす反射値の下限。試走会CSVの白レンジを使う。
const int WHITE_MARK_REFLECTION_MIN = COLOR_WHITE_RANGE.reflection.min;
// 白マークとみなす反射値の上限。
const int WHITE_MARK_REFLECTION_MAX = COLOR_WHITE_RANGE.reflection.max;
// 白マークとみなすHSV彩度Sの上限。白は色味が少ないので低くする。
const int WHITE_MARK_MAX_SATURATION = COLOR_WHITE_RANGE.hsv_s.max;
// 白マークとみなすHSV明度Vの下限。暗い灰色/黒の誤検知を防ぐ。
const int WHITE_MARK_MIN_VALUE = COLOR_WHITE_RANGE.hsv_v.min;
// 白マークとみなすHSV明度Vの上限。
const int WHITE_MARK_MAX_VALUE = COLOR_WHITE_RANGE.hsv_v.max;

// 黒マークとみなす反射値の下限。
const int BLACK_MARK_REFLECTION_MIN = COLOR_BLACK_RANGE.reflection.min;
// 黒マークとみなす反射値の上限。試走会CSVの黒レンジを使う。
const int BLACK_MARK_REFLECTION_MAX = COLOR_BLACK_RANGE.reflection.max;
// 黒マークとみなすHSV彩度Sの上限。色付きマークの誤検知を防ぐ。
const int BLACK_MARK_MAX_SATURATION = COLOR_BLACK_RANGE.hsv_s.max;
// 黒マークとみなすHSV明度Vの下限。
const int BLACK_MARK_MIN_VALUE = COLOR_BLACK_RANGE.hsv_v.min;
// 黒マークとみなすHSV明度Vの上限。明るい灰色/白の誤検知を防ぐ。
const int BLACK_MARK_MAX_VALUE = COLOR_BLACK_RANGE.hsv_v.max;

enum class TurnDirection {
  Left,
  Right
};

struct DriveMotors {
  pup_motor_t *left;
  pup_motor_t *right;
};

}  // namespace etrobo_app

#endif
