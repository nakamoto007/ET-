#ifndef HUB_IMU_CORRECTION_H
#define HUB_IMU_CORRECTION_H

#ifdef __cplusplus
extern "C" {
#endif

extern int hub_imu_calibration_status;

float hub_imu_get_raw_heading(void);
float hub_imu_get_corrected_heading(void);
void hub_imu_clear_heading_drift_correction(void);
void hub_imu_set_heading_drift_rate(float drift_deg_per_min);
float hub_imu_get_heading_drift_rate(void);

#ifdef __cplusplus
}
#endif

#endif
