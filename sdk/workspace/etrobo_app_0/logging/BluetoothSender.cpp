#include "BluetoothSender.h"

#include <cstdio>
#include <cstring>

extern "C" {
#include <pbdrv/bluetooth.h>
}
#include <serial/serial.h>
#include <syssvc/serial.h>

namespace {

const int BLUETOOTH_POLL_PERIOD_US = 100 * 1000;
volatile bool fg_serial_open = false;

bool isUartConnected(void)
{
  return pbdrv_bluetooth_is_connected(PBDRV_BLUETOOTH_CONNECTION_UART);
}

}  // namespace

void bluetooth_sender_start(void)
{
  fg_serial_open = false;
}

void bluetooth_sender_task(intptr_t unused)
{
  (void)unused;

  // Bluetoothシリアルを開くと、ドライバ側で広告開始とUART接続待ちを行う。
  const ER result = serial_opn_por(SIO_BLUETOOTH_PORTID);
  if (result == E_OK || result == E_OBJ) {
    fg_serial_open = true;
  }

  for (;;) {
    dly_tsk(BLUETOOTH_POLL_PERIOD_US);
  }
}

bool bluetooth_sender_is_connected(void)
{
  return isUartConnected();
}

bool bluetooth_sender_is_ready(void)
{
  return fg_serial_open && isUartConnected();
}

bool bluetooth_sender_send(const char *data)
{
  if (data == nullptr || !bluetooth_sender_is_ready()) {
    return false;
  }

  const size_t length = std::strlen(data);
  if (length == 0 || length > 256) {
    return false;
  }

  return serial_wri_dat(SIO_BLUETOOTH_PORTID, data,
                        static_cast<uint_t>(length)) ==
         static_cast<ER_UINT>(length);
}

bool bluetooth_sender_send_int3(int id, int value1, int value2)
{
  char line[32];
  const int length = std::snprintf(line, sizeof(line), "%d,%d,%d\n",
                                   id, value1, value2);
  if (length <= 0 || static_cast<size_t>(length) >= sizeof(line)) {
    return false;
  }
  return bluetooth_sender_send(line);
}
