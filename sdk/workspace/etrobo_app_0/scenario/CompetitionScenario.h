#ifndef COMPETITION_SCENARIO_H
#define COMPETITION_SCENARIO_H

typedef enum {
  COMPETITION_SCENARIO_STATE_START = 0,
  COMPETITION_SCENARIO_STATE_WAIT_FOR_START,
  COMPETITION_SCENARIO_STATE_CALIBRATE_POSE,
  COMPETITION_SCENARIO_STATE_LINE_TRACE_TO_BOTTLE,
  COMPETITION_SCENARIO_STATE_DETECT_COLOR,
  COMPETITION_SCENARIO_STATE_LOWER_ARM,
  COMPETITION_SCENARIO_STATE_LINE_TRACE_AFTER_BOTTLE,
  COMPETITION_SCENARIO_STATE_CHALLENGE_LAPS,
  COMPETITION_SCENARIO_STATE_PUSH_BOTTLE,
  COMPETITION_SCENARIO_STATE_GOAL_RUN,
  COMPETITION_SCENARIO_STATE_FINISHED,
  COMPETITION_SCENARIO_STATE_FAILED,
} competition_scenario_state_t;

#ifdef __cplusplus
extern "C" {
#endif

void competition_scenario_run(void);
void competition_scenario_run_challenge_test(void);
void competition_scenario_set_state(competition_scenario_state_t state);
competition_scenario_state_t competition_scenario_get_state(void);

#ifdef __cplusplus
}
#endif

#endif
