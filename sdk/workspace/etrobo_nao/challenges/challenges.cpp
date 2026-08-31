#include "challenges.h"

#include "app.h"
#include "DriveController.h"
#include "RobotConfig.h"

#include <spike/hub/display.h>

namespace {

const char DEFAULT_CHALLENGE_STEPS[] =
  "FFFFFFFFF";

bool isIgnoredStep(char step)
{
  return step == ' ' || step == '\n' || step == '\r' || step == '\t';
}

int absoluteValue(int value)
{
  return value < 0 ? -value : value;
}

int countSameSteps(const char *steps, int start_index, char target_step)
{
  int count = 0;
  while (steps[start_index + count] == target_step) {
    ++count;
  }
  return count;
}

drive_result_t runChallengeStraight(int distance_mm)
{
  if (absoluteValue(distance_mm) <= 0) {
    return DRIVE_RESULT_OK;
  }

  return drive_straight_profile_mm(
    etrobo_app::CHALLENGE_STEP_START_SPEED_DEG_S,
    etrobo_app::CHALLENGE_STEP_FORWARD_SPEED_DEG_S,
    0,
    etrobo_app::CHALLENGE_STEP_ACCEL_DISTANCE_MM,
    etrobo_app::CHALLENGE_STEP_DECEL_DISTANCE_MM,
    distance_mm);
}

challenge_run_result_t runForwardSteps(int count)
{
  if (count <= 0) {
    return CHALLENGE_RUN_RESULT_OK;
  }

  hub_display_char('F');
  const int distance_mm =
    count * etrobo_app::CHALLENGE_STEP_FORWARD_DISTANCE_MM;
  const drive_result_t result =
    runChallengeStraight(distance_mm);
  if (result != DRIVE_RESULT_OK) {
    stop_motors();
    return CHALLENGE_RUN_RESULT_DRIVE_FAILED;
  }
  return CHALLENGE_RUN_RESULT_OK;
}

challenge_run_result_t runBackwardSteps(int count)
{
  if (count <= 0) {
    return CHALLENGE_RUN_RESULT_OK;
  }

  hub_display_char('B');
  const int distance_mm =
    -count * etrobo_app::CHALLENGE_STEP_FORWARD_DISTANCE_MM;
  const drive_result_t result =
    runChallengeStraight(distance_mm);
  if (result != DRIVE_RESULT_OK) {
    stop_motors();
    return CHALLENGE_RUN_RESULT_DRIVE_FAILED;
  }
  return CHALLENGE_RUN_RESULT_OK;
}

challenge_run_result_t runTurnSteps(char step, int count)
{
  if (count <= 0) {
    return CHALLENGE_RUN_RESULT_OK;
  }

  const int degrees = (step == 'L' ? -90 : 90) * count;
  hub_display_char(step);
  const int result =
    turn(etrobo_app::CHALLENGE_STEP_TURN_SPEED_DEG_S, degrees);
  if (result != TURN_RESULT_OK) {
    stop_motors();
    return CHALLENGE_RUN_RESULT_TURN_FAILED;
  }
  return CHALLENGE_RUN_RESULT_OK;
}

}  // namespace

challenge_run_result_t challenges_run_steps(const char *steps)
{
  if (steps == nullptr) {
    return CHALLENGE_RUN_RESULT_NULL_STEPS;
  }

  bool has_command = false;
  for (int index = 0; steps[index] != '\0';) {
    const char step = steps[index];
    if (isIgnoredStep(step)) {
      ++index;
      continue;
    }

    has_command = true;
    if (step == 'F') {
      const int forward_count = countSameSteps(steps, index, step);
      const challenge_run_result_t result =
        runForwardSteps(forward_count);
      if (result != CHALLENGE_RUN_RESULT_OK) {
        return result;
      }
      index += forward_count;
      continue;
    }

    if (step == 'B') {
      const int backward_count = countSameSteps(steps, index, step);
      const challenge_run_result_t result =
        runBackwardSteps(backward_count);
      if (result != CHALLENGE_RUN_RESULT_OK) {
        return result;
      }
      index += backward_count;
      continue;
    }

    if (step == 'L' || step == 'R') {
      const int turn_count = countSameSteps(steps, index, step);
      const challenge_run_result_t result = runTurnSteps(step, turn_count);
      if (result != CHALLENGE_RUN_RESULT_OK) {
        return result;
      }
      index += turn_count;
      continue;
    }
    if (step == 'C') {
      continue;
    }

    stop_motors();
    return CHALLENGE_RUN_RESULT_INVALID_STEP;
  }

  if (!has_command) {
    return CHALLENGE_RUN_RESULT_EMPTY_STEPS;
  }

  stop_motors();
  return CHALLENGE_RUN_RESULT_OK;
}

challenge_run_result_t challenges_run_default_steps(void)
{
  return challenges_run_steps(DEFAULT_CHALLENGE_STEPS);
}
