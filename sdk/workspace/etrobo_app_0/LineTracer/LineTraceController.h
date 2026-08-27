#ifndef LINE_TRACE_CONTROLLER_H
#define LINE_TRACE_CONTROLLER_H

typedef enum {
  LINE_TRACE_RESULT_OK = 0,
  LINE_TRACE_RESULT_TIMEOUT = 2,
  LINE_TRACE_RESULT_DEVICE_ERROR = -1,
} line_trace_result_t;

#ifdef __cplusplus
extern "C" {
#endif

line_trace_result_t line_trace_step(void);
line_trace_result_t run_line_trace_cycles(int cycles);
line_trace_result_t run_line_trace_mm(int distance_mm);
void stop_line_trace(void);

#ifdef __cplusplus
}
#endif

#endif
