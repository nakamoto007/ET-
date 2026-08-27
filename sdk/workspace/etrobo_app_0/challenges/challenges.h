#ifndef CHALLENGES_H
#define CHALLENGES_H

typedef enum {
  CHALLENGE_RUN_RESULT_OK = 0,
  CHALLENGE_RUN_RESULT_INVALID_STEP = 1,
  CHALLENGE_RUN_RESULT_DRIVE_FAILED = 2,
  CHALLENGE_RUN_RESULT_TURN_FAILED = 3,
  CHALLENGE_RUN_RESULT_EMPTY_STEPS = 4,
  CHALLENGE_RUN_RESULT_NULL_STEPS = -1,
} challenge_run_result_t;

#ifdef __cplusplus
extern "C" {
#endif

challenge_run_result_t challenges_run_steps(const char *steps);
challenge_run_result_t challenges_run_default_steps(void);

#ifdef __cplusplus
}
#endif

#endif
