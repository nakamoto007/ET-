#include "kernel_cfg.h"
#include "ColorSensorService.h"
#include "RobotConfig.h"

#include <kernel.h>
#include <spike/pup/colorsensor.h>

namespace {

constexpr uint16_t COLOR_RGB_MAX = 1023;

pup_device_t *color_sensor = nullptr;
color_sensor_values_t cached_values = {};
bool reflection_only_mode = false;
int32_t normalize_black_reflection =
  etrobo_app::COLOR_SENSOR_DEFAULT_NORMALIZE_BLACK_REFLECTION;
int32_t normalize_white_reflection =
  etrobo_app::COLOR_SENSOR_DEFAULT_NORMALIZE_WHITE_REFLECTION;

int32_t normalizeReflectionWithRange(int32_t reflection,
                                     int32_t black_reflection,
                                     int32_t white_reflection)
{
  const int32_t range = white_reflection - black_reflection;
  if (range <= 0) {
    return etrobo_app::COLOR_SENSOR_NORMALIZED_TARGET_REFLECTION;
  }

  int32_t normalized = ((reflection - black_reflection) * 100) / range;
  if (normalized < etrobo_app::COLOR_SENSOR_NORMALIZED_REFLECTION_MIN) {
    normalized = etrobo_app::COLOR_SENSOR_NORMALIZED_REFLECTION_MIN;
  }
  if (normalized > etrobo_app::COLOR_SENSOR_NORMALIZED_REFLECTION_MAX) {
    normalized = etrobo_app::COLOR_SENSOR_NORMALIZED_REFLECTION_MAX;
  }
  return normalized;
}

int32_t normalizeReflectionValue(int32_t reflection)
{
  int32_t black_reflection;
  int32_t white_reflection;
  loc_cpu();
  black_reflection = normalize_black_reflection;
  white_reflection = normalize_white_reflection;
  unl_cpu();
  return normalizeReflectionWithRange(reflection,
                                      black_reflection,
                                      white_reflection);
}

void convertRgbToHsv(const pup_color_rgb_t &rgb, color_sensor_values_t *values)
{
  const uint16_t max_value =
    rgb.r > rgb.g ? (rgb.r > rgb.b ? rgb.r : rgb.b)
                  : (rgb.g > rgb.b ? rgb.g : rgb.b);
  const uint16_t min_value =
    rgb.r < rgb.g ? (rgb.r < rgb.b ? rgb.r : rgb.b)
                  : (rgb.g < rgb.b ? rgb.g : rgb.b);
  const uint16_t delta = max_value - min_value;

  values->hsv_v = static_cast<uint8_t>(
    (static_cast<uint32_t>(max_value) * 100 + COLOR_RGB_MAX / 2) /
    COLOR_RGB_MAX);
  values->hsv_v8 = static_cast<uint8_t>(
    (static_cast<uint32_t>(max_value) * 255 + COLOR_RGB_MAX / 2) /
    COLOR_RGB_MAX);
  if (delta == 0) {
    values->hsv_h = 0;
    values->hsv_s = 0;
    return;
  }

  values->hsv_s = static_cast<uint8_t>(
    (static_cast<uint32_t>(delta) * 100 + max_value / 2) / max_value);

  int hue;
  if (max_value == rgb.r) {
    hue = 60 * (static_cast<int>(rgb.g) - static_cast<int>(rgb.b)) / delta;
  } else if (max_value == rgb.g) {
    hue = 120 + 60 * (static_cast<int>(rgb.b) - static_cast<int>(rgb.r)) /
                      delta;
  } else {
    hue = 240 + 60 * (static_cast<int>(rgb.r) - static_cast<int>(rgb.g)) /
                      delta;
  }
  if (hue < 0) {
    hue += 360;
  }
  values->hsv_h = static_cast<uint16_t>(hue);
}

void storeCachedValues(const color_sensor_values_t &values)
{
  loc_cpu();
  cached_values = values;
  unl_cpu();
}

color_sensor_values_t loadCachedValues(void)
{
  color_sensor_values_t values = {};
  loc_cpu();
  values = cached_values;
  unl_cpu();
  return values;
}

bool ensureColorSensor(void)
{
  if (color_sensor == nullptr) {
    color_sensor = pup_color_sensor_get_device(etrobo_app::COLOR_SENSOR_PORT);
  }
  return color_sensor != nullptr;
}

bool loadReflectionOnlyMode(void)
{
  bool enabled;
  loc_cpu();
  enabled = reflection_only_mode;
  unl_cpu();
  return enabled;
}

bool updateReflectionLocked(void)
{
  color_sensor_values_t next = loadCachedValues();
  next.color_sensor_ready = ensureColorSensor();
  if (!next.color_sensor_ready) {
    storeCachedValues(next);
    return false;
  }

  pup_color_sensor_light_on(color_sensor);
  next.reflection = pup_color_sensor_reflection(color_sensor);
  next.normalized_reflection = normalizeReflectionValue(next.reflection);

  storeCachedValues(next);
  return true;
}

}  // namespace

bool color_sensor_service_update(void)
{
  if (!color_sensor_service_lock()) {
    return false;
  }

  if (loadReflectionOnlyMode()) {
    const bool updated = updateReflectionLocked();
    color_sensor_service_unlock();
    return updated;
  }

  color_sensor_values_t next = {};
  next.color_sensor_ready = ensureColorSensor();
  if (!next.color_sensor_ready) {
    storeCachedValues(next);
    color_sensor_service_unlock();
    return false;
  }

  pup_color_sensor_light_on(color_sensor);
  next.reflection = pup_color_sensor_reflection(color_sensor);
  next.normalized_reflection = normalizeReflectionValue(next.reflection);
  const pup_color_rgb_t rgb = pup_color_sensor_rgb(color_sensor);

  // 環境光は別モード読みになるため、走行中はRGB由来の値だけに統一する。
  next.ambient = -1;
  next.rgb_r = rgb.r;
  next.rgb_g = rgb.g;
  next.rgb_b = rgb.b;
  convertRgbToHsv(rgb, &next);

  storeCachedValues(next);
  color_sensor_service_unlock();
  return true;
}

bool color_sensor_service_update_reflection(void)
{
  if (!color_sensor_service_lock()) {
    return false;
  }

  const bool updated = updateReflectionLocked();
  color_sensor_service_unlock();
  return updated;
}

void color_sensor_service_set_reflection_only(bool enabled)
{
  loc_cpu();
  reflection_only_mode = enabled;
  unl_cpu();
}

bool color_sensor_service_is_reflection_only(void)
{
  return loadReflectionOnlyMode();
}

bool color_sensor_service_lock(void)
{
  return wai_sem(COLOR_SENSOR_SERVICE_SEM) == E_OK;
}

void color_sensor_service_unlock(void)
{
  (void)sig_sem(COLOR_SENSOR_SERVICE_SEM);
}

bool color_sensor_service_set_normalization_reflection(
  int32_t black_reflection,
  int32_t white_reflection)
{
  if (white_reflection <= black_reflection) {
    return false;
  }

  loc_cpu();
  normalize_black_reflection = black_reflection;
  normalize_white_reflection = white_reflection;
  if (cached_values.color_sensor_ready) {
    cached_values.normalized_reflection =
      normalizeReflectionWithRange(cached_values.reflection,
                                   normalize_black_reflection,
                                   normalize_white_reflection);
  }
  unl_cpu();
  return true;
}

int32_t color_sensor_service_normalize_reflection(int32_t reflection)
{
  return normalizeReflectionValue(reflection);
}

void color_sensor_service_store_reflection(int32_t reflection)
{
  const int32_t normalized_reflection =
    color_sensor_service_normalize_reflection(reflection);

  loc_cpu();
  cached_values.color_sensor_ready = true;
  cached_values.reflection = reflection;
  cached_values.normalized_reflection = normalized_reflection;
  unl_cpu();
}

bool color_sensor_service_get_values(color_sensor_values_t *values)
{
  if (values == nullptr) {
    return false;
  }

  loc_cpu();
  *values = cached_values;
  unl_cpu();
  return values->color_sensor_ready;
}
