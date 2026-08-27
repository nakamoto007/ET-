#ifndef DRIVE_BASE_H
#define DRIVE_BASE_H

#include "RobotConfig.h"

#include <cstdint>

namespace etrobo_app {

struct StraightCorrectionState {
  int32_t left_start_count;
  int32_t right_start_count;
  double start_heading;
};

int clampMotorSpeed(double speed_deg_s);
bool getDriveMotors(DriveMotors *motors);
bool setupDriveMotor(pup_motor_t *motor, pup_direction_t direction);
double encoderDegreesToMm(int32_t degrees);
double mmToEncoderDegrees(double mm);
double averageWheelDistanceMm(double left_mm, double right_mm);
void resetDriveMotorCounts(const DriveMotors &motors);
void setStraightSpeed(const DriveMotors &motors, int speed_deg_s);
void setMotorSpeeds(const DriveMotors &motors,
                    double left_speed_deg_s, double right_speed_deg_s);
StraightCorrectionState beginStraightCorrection(const DriveMotors &motors);
double calculateStraightCorrection(const DriveMotors &motors,
                                   const StraightCorrectionState &state,
                                   int drive_direction);
void setCorrectedStraightSpeed(const DriveMotors &motors,
                               int base_speed_deg_s,
                               const StraightCorrectionState &state);
void brakeMotors(const DriveMotors &motors);
void stopDriveMotors(const DriveMotors &motors);

}  // namespace etrobo_app

#endif
