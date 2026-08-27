#include "ColorDetector.h"
#include "ColorSensorService.h"
#include "RobotConfig.h"

namespace {

color_detector_status_t status = {};

int minimumInt(int left, int right)
{
  return left < right ? left : right;
}

int maximumInt(int left, int right)
{
  return left > right ? left : right;
}

bool isInRange(int value, int min_value, int max_value)
{
  return value >= minimumInt(min_value, max_value) &&
         value <= maximumInt(min_value, max_value);
}

bool isHueInRange(int hue, int min_hue, int max_hue)
{
  if (min_hue <= max_hue) {
    return hue >= min_hue && hue <= max_hue;
  }
  return hue >= min_hue || hue <= max_hue;
}

int absoluteInt(int value)
{
  return value < 0 ? -value : value;
}

bool matchesReflection(int reflection, int min_value, int max_value)
{
  return isInRange(reflection, min_value, max_value);
}

bool matchesNormalizedReflection(int normalized_reflection,
                                 int raw_min_value,
                                 int raw_max_value)
{
  return matchesReflection(
      normalized_reflection,
      color_sensor_service_normalize_reflection(raw_min_value),
      color_sensor_service_normalize_reflection(raw_max_value));
}

bool matchesLowSaturation(int saturation, int max_saturation)
{
  return saturation <= max_saturation;
}

bool matchesValue(int value, int min_value, int max_value)
{
  return isInRange(value, min_value, max_value);
}

bool matchesRed(const color_detector_status_t &values)
{
  const int r = static_cast<int>(values.rgb_r);
  const int g = static_cast<int>(values.rgb_g);
  const int b = static_cast<int>(values.rgb_b);
  return isInRange(r, etrobo_app::RED_MARK_R_MIN,
                   etrobo_app::RED_MARK_R_MAX) &&
         isInRange(g, etrobo_app::RED_MARK_G_MIN,
                   etrobo_app::RED_MARK_G_MAX) &&
         isInRange(b, etrobo_app::RED_MARK_B_MIN,
                   etrobo_app::RED_MARK_B_MAX) &&
         r - g >= etrobo_app::RED_MARK_RGB_DIFFERENCE_MIN &&
         r - b >= etrobo_app::RED_MARK_RGB_DIFFERENCE_MIN;
}

bool matchesBlue(const color_detector_status_t &values)
{
  const int r = static_cast<int>(values.rgb_r);
  const int g = static_cast<int>(values.rgb_g);
  const int b = static_cast<int>(values.rgb_b);
  return isInRange(r, etrobo_app::BLUE_MARK_R_MIN,
                   etrobo_app::BLUE_MARK_R_MAX) &&
         isInRange(g, etrobo_app::BLUE_MARK_G_MIN,
                   etrobo_app::BLUE_MARK_G_MAX) &&
         isInRange(b, etrobo_app::BLUE_MARK_B_MIN,
                   etrobo_app::BLUE_MARK_B_MAX) &&
         b - r >= etrobo_app::BLUE_MARK_RGB_DIFFERENCE_MIN &&
         b - g >= etrobo_app::BLUE_MARK_RGB_DIFFERENCE_MIN;
}

bool matchesYellow(const color_detector_status_t &values)
{
  const int r = static_cast<int>(values.rgb_r);
  const int g = static_cast<int>(values.rgb_g);
  const int b = static_cast<int>(values.rgb_b);
  return isInRange(r, etrobo_app::YELLOW_MARK_R_MIN,
                   etrobo_app::YELLOW_MARK_R_MAX) &&
         isInRange(g, etrobo_app::YELLOW_MARK_G_MIN,
                   etrobo_app::YELLOW_MARK_G_MAX) &&
         isInRange(b, etrobo_app::YELLOW_MARK_B_MIN,
                   etrobo_app::YELLOW_MARK_B_MAX) &&
         r - b >= etrobo_app::YELLOW_MARK_RGB_DIFFERENCE_MIN &&
         g - b >= etrobo_app::YELLOW_MARK_RGB_DIFFERENCE_MIN &&
         absoluteInt(r - g) <= etrobo_app::YELLOW_MARK_RGB_BALANCE_MAX;
}

bool matchesGreen(const color_detector_status_t &values)
{
  return isHueInRange(values.hsv_h,
                      etrobo_app::COLOR_GREEN_RANGE.hsv_h.min,
                      etrobo_app::COLOR_GREEN_RANGE.hsv_h.max) &&
         isInRange(values.hsv_s,
                   etrobo_app::COLOR_GREEN_RANGE.hsv_s.min,
                   etrobo_app::COLOR_GREEN_RANGE.hsv_s.max) &&
         isInRange(values.hsv_v,
                   etrobo_app::COLOR_GREEN_RANGE.hsv_v.min,
                   etrobo_app::COLOR_GREEN_RANGE.hsv_v.max);
}

detected_color_t classifyColor(const color_detector_status_t &values)
{
  if (matchesRed(values)) {
    return COLOR_DETECT_RED;
  }
  if (matchesBlue(values)) {
    return COLOR_DETECT_BLUE;
  }
  if (matchesYellow(values)) {
    return COLOR_DETECT_YELLOW;
  }
  if (matchesGreen(values)) {
    return COLOR_DETECT_GREEN;
  }

  if (matchesNormalizedReflection(values.normalized_reflection,
                                  etrobo_app::BLACK_MARK_REFLECTION_MIN,
                                  etrobo_app::BLACK_MARK_REFLECTION_MAX) &&
      matchesValue(values.hsv_v,
                   etrobo_app::BLACK_MARK_MIN_VALUE,
                   etrobo_app::BLACK_MARK_MAX_VALUE)) {
    return COLOR_DETECT_BLACK;
  }

  if (matchesNormalizedReflection(values.normalized_reflection,
                                  etrobo_app::GRAY_MARK_REFLECTION_MIN,
                                  etrobo_app::GRAY_MARK_REFLECTION_MAX) &&
      matchesLowSaturation(values.hsv_s,
                           etrobo_app::GRAY_MARK_MAX_SATURATION) &&
      matchesValue(values.hsv_v,
                   etrobo_app::GRAY_MARK_MIN_VALUE,
                   etrobo_app::GRAY_MARK_MAX_VALUE)) {
    return COLOR_DETECT_GRAY;
  }

  if (matchesNormalizedReflection(values.normalized_reflection,
                                  etrobo_app::WHITE_MARK_REFLECTION_MIN,
                                  etrobo_app::WHITE_MARK_REFLECTION_MAX) &&
      matchesLowSaturation(values.hsv_s,
                           etrobo_app::WHITE_MARK_MAX_SATURATION) &&
      matchesValue(values.hsv_v,
                   etrobo_app::WHITE_MARK_MIN_VALUE,
                   etrobo_app::WHITE_MARK_MAX_VALUE)) {
    return COLOR_DETECT_WHITE;
  }

  return COLOR_DETECT_UNKNOWN;
}

}  // namespace

void color_detector_step(void)
{
  color_detector_status_t next = {};
  color_sensor_values_t values = {};
  if (!color_sensor_service_get_values(&values)) {
    status = next;
    return;
  }

  next.ready = true;
  next.reflection = values.reflection;
  next.normalized_reflection = values.normalized_reflection;
  next.rgb_r = values.rgb_r;
  next.rgb_g = values.rgb_g;
  next.rgb_b = values.rgb_b;
  next.hsv_h = values.hsv_h;
  next.hsv_s = values.hsv_s;
  next.hsv_v = values.hsv_v;
  next.hsv_v8 = values.hsv_v8;
  next.color = classifyColor(next);
  status = next;
}

color_detector_status_t color_detector_get_status(void)
{
  return status;
}

const char *color_detector_name(detected_color_t color)
{
  switch (color) {
  case COLOR_DETECT_BLACK:
    return "black";
  case COLOR_DETECT_GRAY:
    return "gray";
  case COLOR_DETECT_WHITE:
    return "white";
  case COLOR_DETECT_RED:
    return "red";
  case COLOR_DETECT_BLUE:
    return "blue";
  case COLOR_DETECT_YELLOW:
    return "yellow";
  case COLOR_DETECT_GREEN:
    return "green";
  case COLOR_DETECT_UNKNOWN:
  default:
    return "unknown";
  }
}
