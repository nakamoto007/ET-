#ifndef COLOR_DETECTOR_H
#define COLOR_DETECTOR_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
  COLOR_DETECT_UNKNOWN = 0,
  COLOR_DETECT_BLACK,
  COLOR_DETECT_GRAY,
  COLOR_DETECT_WHITE,
  COLOR_DETECT_RED,
  COLOR_DETECT_BLUE,
  COLOR_DETECT_YELLOW,
  COLOR_DETECT_GREEN,
} detected_color_t;

typedef struct {
  bool ready;
  detected_color_t color;
  int32_t reflection;
  int32_t normalized_reflection;
  uint16_t rgb_r;
  uint16_t rgb_g;
  uint16_t rgb_b;
  uint16_t hsv_h;
  uint8_t hsv_s;
  uint8_t hsv_v;
  uint8_t hsv_v8;
} color_detector_status_t;

#ifdef __cplusplus
extern "C" {
#endif

void color_detector_step(void);
color_detector_status_t color_detector_get_status(void);
const char *color_detector_name(detected_color_t color);

#ifdef __cplusplus
}
#endif

#endif
