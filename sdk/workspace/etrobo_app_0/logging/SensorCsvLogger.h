#ifndef SENSOR_CSV_LOGGER_H
#define SENSOR_CSV_LOGGER_H

#ifdef __cplusplus
extern "C" {
#endif

void sensor_csv_logger_print_header(void);
void sensor_csv_logger_print_row(int elapsed_ms);
void sensor_csv_logger_flush(void);
void run_sensor_csv_logger_seconds(int seconds);

#ifdef __cplusplus
}
#endif

#endif
