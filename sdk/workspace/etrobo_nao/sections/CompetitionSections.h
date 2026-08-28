#ifndef COMPETITION_SECTIONS_H
#define COMPETITION_SECTIONS_H

#include "ColorDetector.h"

#include <stdbool.h>

typedef struct {
  bool ok;
  char step;
  char reason;
} competition_section_result_t;

static inline competition_section_result_t competition_section_ok(void)
{
  competition_section_result_t result = { true, 'O', 'O' };
  return result;
}

static inline competition_section_result_t competition_section_fail(
  char step,
  char reason)
{
  competition_section_result_t result = { false, step, reason };
  return result;
}

#ifdef __cplusplus
extern "C" {
#endif

competition_section_result_t run_line_trace_to_bottle_section(void);
competition_section_result_t run_bottle_color_carry_section(void);
detected_color_t bottle_color_carry_section_get_color(void);
competition_section_result_t run_challenge_section(void);
competition_section_result_t run_bottle_push_section(void);
competition_section_result_t run_goal_section(void);

#ifdef __cplusplus
}
#endif

#endif
