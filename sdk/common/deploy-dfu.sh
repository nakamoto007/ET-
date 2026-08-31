#!/bin/bash
set -euo pipefail

HERE=$( cd "$( dirname "$0" )" && pwd -P )

if [ -n "${PYTHON3:-}" ]; then
  :
elif [ -n "${ETROBO_SPIKE_RT_TOOLS:-}" ] && [ -x "$ETROBO_SPIKE_RT_TOOLS/python/bin/python3" ]; then
  PYTHON3="$ETROBO_SPIKE_RT_TOOLS/python/bin/python3"
elif command -v python.exe > /dev/null 2>&1; then
  PYTHON3=python.exe
else
  PYTHON3=python3
fi

MPTOP=$HERE/../../asp3/target/primehub_gcc/
DFU=$MPTOP/tools/dfu.py
PYDFU=$MPTOP/tools/pydfu.py

TEXT0_ADDR=0x8008000
DFU_VID=0x0694
DFU_PID=0x0008

if [ $# -ne 1 ]; then
  echo "Usage: $0 <asp.bin>" >&2
  exit 2
fi

if [ ! -f "$1" ]; then
  echo "$1 was not found" >&2
  exit 2
fi

trap 'rm -f firmware.dfu' EXIT

echo "DFU Create $1"
$PYTHON3 $DFU -b $TEXT0_ADDR:$1 firmware.dfu

echo "Writing $1 to the board"
$PYTHON3 $PYDFU -u firmware.dfu --vid $DFU_VID --pid $DFU_PID
