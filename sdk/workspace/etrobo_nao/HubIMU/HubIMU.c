#include <math.h>
#include <stdbool.h>
#include <kernel.h>
#include <pbio/imu.h>
#include <spike/hub/imu.h>

#include "HubIMUCorrection.h"

int hub_imu_calibration_status = 0;
pbio_imu_persistent_settings_t hub_imu_settings;

static float heading_drift_deg_per_sec = 0.0f;
static SYSTIM heading_drift_base_time = 0;
static float heading_drift_base_raw_heading = 0.0f;
static bool heading_drift_correction_enabled = false;

static float normalize_heading(float heading)
{
  while (heading > 180.0f) {
    heading -= 360.0f;
  }
  while (heading < -180.0f) {
    heading += 360.0f;
  }
  return heading;
}

static float read_raw_heading(void)
{
  return pbio_imu_get_heading(PBIO_IMU_HEADING_TYPE_3D);
}

static void reset_heading_drift_base(void)
{
  SYSTIM now = 0;
  if (get_tim(&now) != E_OK) {
    return;
  }
  heading_drift_base_time = now;
  heading_drift_base_raw_heading = read_raw_heading();
}

static bool looks_ok(pbio_imu_persistent_settings_t *settings)
{
  if (settings->gyro_stationary_threshold < 0.0f) {
    return false;
  }
  if (settings->gyro_stationary_threshold > 5.0f) {
    return false;
  }
  if (settings->accel_stationary_threshold < 2000.0f) {
    return false;
  }
  if (settings->accel_stationary_threshold > 3000.0f) {
    return false;
  }
  return true;
}

pbio_error_t hub_imu_init(void)
{
  pbio_imu_persistent_settings_t *settings_in_flash = NULL;

  pbio_imu_init();
  if (pbio_imu_get_settings(&settings_in_flash) == PBIO_SUCCESS) {
    if (looks_ok(settings_in_flash)) {
      hub_imu_calibration_status = 1;
      hub_imu_settings = *settings_in_flash;
      pbio_imu_apply_loaded_settings(settings_in_flash);
      return PBIO_SUCCESS;
    }
    hub_imu_calibration_status = -1;
  } else {
    hub_imu_calibration_status = -2;
  }

  hub_imu_settings.flags = 0;
  hub_imu_settings.gyro_stationary_threshold = 2.0f;
  hub_imu_settings.accel_stationary_threshold = 2500.0f;
  hub_imu_settings.gravity_pos.x = 9868.006f;
  hub_imu_settings.gravity_neg.x = -9966.395f;
  hub_imu_settings.gravity_pos.y = 9629.595f;
  hub_imu_settings.gravity_neg.y = -9784.85f;
  hub_imu_settings.gravity_pos.z = 9843.534f;
  hub_imu_settings.gravity_neg.z = -9930.068f;
  hub_imu_settings.angular_velocity_bias_start.x = 0.16546678f;
  hub_imu_settings.angular_velocity_bias_start.y = -1.2014996f;
  hub_imu_settings.angular_velocity_bias_start.z = 0.365810432f;
  hub_imu_settings.angular_velocity_scale.x = 362.1383f;
  hub_imu_settings.angular_velocity_scale.y = 357.9202f;
  hub_imu_settings.angular_velocity_scale.z = 361.173868f;
  hub_imu_settings.heading_correction_1d = 360.0f;
  pbio_imu_apply_loaded_settings(&hub_imu_settings);

  return PBIO_SUCCESS;
}

bool hub_imu_is_ready(void)
{
  return pbio_imu_is_ready();
}

bool hub_imu_is_stationary(void)
{
  return pbio_imu_is_stationary();
}

void hub_imu_set_tilt(float angle)
{
  double tilt = (double)angle * M_PI / 180.0f;
  double sin_tilt = sin(tilt);
  double cos_tilt = cos(tilt);
  pbio_geometry_xyz_t front = { .x = cos_tilt, .y = 0.0f, .z = sin_tilt };
  pbio_geometry_xyz_t top = { .x = -sin_tilt, .y = 0.0f, .z = cos_tilt };

  pbio_imu_set_base_orientation(&front, &top);
}

void hub_imu_get_acceleration(float accel[3])
{
  pbio_imu_get_acceleration((pbio_geometry_xyz_t *)accel, true);
}

void hub_imu_get_angular_velocity(float angv[3])
{
  pbio_imu_get_angular_velocity((pbio_geometry_xyz_t *)angv, true);
}

float hub_imu_get_temperature(void)
{
  return 0.0f;
}

float hub_imu_get_raw_heading(void)
{
  return read_raw_heading();
}

void hub_imu_clear_heading_drift_correction(void)
{
  heading_drift_deg_per_sec = 0.0f;
  heading_drift_correction_enabled = false;
  reset_heading_drift_base();
}

void hub_imu_set_heading_drift_rate(float drift_deg_per_min)
{
  SYSTIM now = 0;
  if (get_tim(&now) != E_OK) {
    heading_drift_correction_enabled = false;
    return;
  }

  heading_drift_deg_per_sec = drift_deg_per_min / 60.0f;
  heading_drift_base_time = now;
  heading_drift_base_raw_heading = read_raw_heading();
  heading_drift_correction_enabled = true;
}

float hub_imu_get_heading_drift_rate(void)
{
  return heading_drift_deg_per_sec * 60.0f;
}

float hub_imu_get_corrected_heading(void)
{
  const float raw_heading = read_raw_heading();
  if (!heading_drift_correction_enabled) {
    return raw_heading;
  }

  SYSTIM now = 0;
  if (get_tim(&now) != E_OK) {
    return raw_heading;
  }

  const float elapsed_sec =
    (float)((double)(now - heading_drift_base_time) / 1000000.0);
  const float raw_delta =
    normalize_heading(raw_heading - heading_drift_base_raw_heading);
  const float drift = heading_drift_deg_per_sec * elapsed_sec;
  return normalize_heading(raw_delta - drift);
}

float hub_imu_get_heading(void)
{
  return hub_imu_get_corrected_heading();
}

void hub_imu_reset_heading(void)
{
  pbio_imu_set_heading(0.0f);
  reset_heading_drift_base();
}
