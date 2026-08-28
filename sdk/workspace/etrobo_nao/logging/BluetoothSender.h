#ifndef BLUETOOTH_SENDER_H
#define BLUETOOTH_SENDER_H

#include <kernel.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void bluetooth_sender_start(void);
void bluetooth_sender_task(intptr_t unused);
bool bluetooth_sender_is_connected(void);
bool bluetooth_sender_is_ready(void);
bool bluetooth_sender_send(const char *data);
bool bluetooth_sender_send_int3(int id, int value1, int value2);

#ifdef __cplusplus
}
#endif

#endif
