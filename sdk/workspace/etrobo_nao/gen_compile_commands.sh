#!/usr/bin/env bash
set -eu

APP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
SPIKE_RT_ROOT="$(cd "$APP_DIR/../../.." && pwd -P)"
ETROBO_ROOT="$(cd "$SPIKE_RT_ROOT/.." && pwd -P)"
WORKSPACE_DIR="$SPIKE_RT_ROOT/sdk/workspace"

if [ -n "${ETROBO_TARGET_GCC:-}" ]; then
  if [ -d "$ETROBO_TARGET_GCC/bin" ]; then
    TOOLCHAIN_DIR="$ETROBO_TARGET_GCC/bin"
  else
    TOOLCHAIN_DIR="$ETROBO_TARGET_GCC"
  fi
else
  TOOLCHAIN_DIR="$ETROBO_ROOT/gcc-arm-none-eabi-10.3-2021.10/bin"
  if [ ! -d "$TOOLCHAIN_DIR" ]; then
    TOOLCHAIN_DIR="$SPIKE_RT_ROOT/gcc-arm-none-eabi-10.3-2021.10/bin"
  fi
fi

emit_entry() {
  local compiler="$1"
  local language="$2"
  local source="$3"
  local object="$4"
  local comma="$5"

  cat <<EOF
  {
    "directory": "$APP_DIR/build",
    "file": "$source",
    "arguments": [
      "$compiler",
      "-x",
      "$language",
      "-g",
      "-O2",
      "-Wall",
      "-DSPIKERT",
      "-I$APP_DIR/build",
      "-I$SPIKE_RT_ROOT/asp3/include",
      "-I$SPIKE_RT_ROOT/asp3",
      "-I$SPIKE_RT_ROOT/asp3/target/primehub_gcc",
      "-I$SPIKE_RT_ROOT/asp3/target/primehub_gcc/stm32fcube",
      "-I$SPIKE_RT_ROOT/asp3/arch/arm_m_gcc/stm32f4xx_stm32cube",
      "-I$SPIKE_RT_ROOT/asp3/arch/arm_m_gcc/stm32f4xx_stm32cube/STM32F4xx_HAL_Driver/Inc",
      "-I$SPIKE_RT_ROOT/asp3/arch/arm_m_gcc/stm32f4xx_stm32cube/CMSIS/Device/ST/STM32F4xx/Include",
      "-I$SPIKE_RT_ROOT/asp3/arch/arm_m_gcc/stm32f4xx_stm32cube/CMSIS/Include",
      "-I$SPIKE_RT_ROOT/asp3/arch/gcc",
      "-I$SPIKE_RT_ROOT/drivers",
      "-I$SPIKE_RT_ROOT/drivers/spike",
      "-I$SPIKE_RT_ROOT/drivers/include",
      "-I$SPIKE_RT_ROOT/drivers/libcpp",
      "-I$SPIKE_RT_ROOT/drivers/libcpp/spike",
      "-I$SPIKE_RT_ROOT/external/libpybricks/lib/pbio/include",
      "-I$SPIKE_RT_ROOT/external/libpybricks/lib/lego",
      "-I$SPIKE_RT_ROOT/external/libpybricks/lib/pbio/platform/prime_hub_spike-rt",
      "-I$APP_DIR",
      "-I$APP_DIR/config",
      "-I$APP_DIR/control",
      "-I$APP_DIR/drive",
      "-I$APP_DIR/HubIMU",
      "-I$APP_DIR/LineTracer",
      "-I$APP_DIR/logging",
      "-I$APP_DIR/sensors",
      "-I$WORKSPACE_DIR",
      "-I$SPIKE_RT_ROOT/asp3/tecsgen",
      "-I$SPIKE_RT_ROOT/asp3/tecs_kernel",
      "-I$SPIKE_RT_ROOT/sdk/common/compat",
      "-I$SPIKE_RT_ROOT/asp3/arch/arm_m_gcc/common",
      "-c",
      "$source",
      "-o",
      "$object"
    ]
  }$comma
EOF
}

{
  echo "["
  emit_entry "$TOOLCHAIN_DIR/arm-none-eabi-g++" "c++" "$APP_DIR/sensors/AllSensors.cpp" "AllSensors.o" ","
  emit_entry "$TOOLCHAIN_DIR/arm-none-eabi-g++" "c++" "$APP_DIR/app.cpp" "app.o" ","
  emit_entry "$TOOLCHAIN_DIR/arm-none-eabi-g++" "c++" "$APP_DIR/logging/BluetoothSender.cpp" "BluetoothSender.o" ","
  emit_entry "$TOOLCHAIN_DIR/arm-none-eabi-g++" "c++" "$APP_DIR/sensors/ColorDetector.cpp" "ColorDetector.o" ","
  emit_entry "$TOOLCHAIN_DIR/arm-none-eabi-g++" "c++" "$APP_DIR/sensors/ColorSensorService.cpp" "ColorSensorService.o" ","
  emit_entry "$TOOLCHAIN_DIR/arm-none-eabi-g++" "c++" "$APP_DIR/drive/DriveBase.cpp" "DriveBase.o" ","
  emit_entry "$TOOLCHAIN_DIR/arm-none-eabi-g++" "c++" "$APP_DIR/drive/DriveController.cpp" "DriveController.o" ","
  emit_entry "$TOOLCHAIN_DIR/arm-none-eabi-g++" "c++" "$APP_DIR/LineTracer/LineTraceController.cpp" "LineTraceController.o" ","
  emit_entry "$TOOLCHAIN_DIR/arm-none-eabi-g++" "c++" "$APP_DIR/sensors/RobotSensors.cpp" "RobotSensors.o" ","
  emit_entry "$TOOLCHAIN_DIR/arm-none-eabi-g++" "c++" "$APP_DIR/control/RobotController.cpp" "RobotController.o" ","
  emit_entry "$TOOLCHAIN_DIR/arm-none-eabi-g++" "c++" "$APP_DIR/control/RobotStateController.cpp" "RobotStateController.o" ","
  emit_entry "$TOOLCHAIN_DIR/arm-none-eabi-g++" "c++" "$APP_DIR/sections/CompetitionSections.cpp" "CompetitionSections.o" ","
  emit_entry "$TOOLCHAIN_DIR/arm-none-eabi-g++" "c++" "$APP_DIR/logging/SensorCsvLogger.cpp" "SensorCsvLogger.o" ","
  emit_entry "$TOOLCHAIN_DIR/arm-none-eabi-g++" "c++" "$APP_DIR/control/SensorLiftController.cpp" "SensorLiftController.o" ","
  emit_entry "$TOOLCHAIN_DIR/arm-none-eabi-g++" "c++" "$APP_DIR/sensors/UltrasonicSensor.cpp" "UltrasonicSensor.o" ","
  emit_entry "$TOOLCHAIN_DIR/arm-none-eabi-gcc" "c" "$APP_DIR/HubIMU/HubIMU.c" "HubIMU.o" ""
  echo "]"
} > "$APP_DIR/compile_commands.json"

echo "generated $APP_DIR/compile_commands.json"
