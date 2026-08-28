#ifndef LOG_CONFIG_H
#define LOG_CONFIG_H

namespace etrobo_app {

// trueなら起動後にBluetoothログ接続ができるまで待つ。接続待ちはHubにBを表示する。
const bool WAIT_FOR_BLUETOOTH_LOG = true;
// CSVセンサーログの送信周期。100ms = 10Hz。Bluetoothログが詰まりにくい周期にする。
const int SENSOR_CSV_LOG_INTERVAL_US = 100 * 1000;

}  // namespace etrobo_app

#endif
