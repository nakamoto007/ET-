APP_DIR := $(abspath $(dir $(lastword $(MAKEFILE_LIST))))
APP := $(notdir $(patsubst %/,%,$(APP_DIR)))
WORKSPACE := $(abspath $(APP_DIR)/..)
COMMON_DIR := $(abspath $(WORKSPACE)/../common)
APP_ENV := _APP_DIR="$(APP_DIR)"; \
	_APP_REAL_DIR="$$(cd "$$_APP_DIR" && pwd -P)"; \
	_SPIKE_RT_ROOT="$$(cd "$$_APP_REAL_DIR/../../.." && pwd -P)"; \
	_ETROBO_ROOT="$$(cd "$$_APP_REAL_DIR/../../../.." && pwd -P)"; \
	if [ -f "$$_ETROBO_ROOT/scripts/etroboenv.sh" ]; then \
		export ETROBO_ROOT="$$_ETROBO_ROOT"; \
		source "$$ETROBO_ROOT/scripts/etroboenv.sh" >/dev/null; \
	else \
		export ETROBO_ENV_MODE="SPIKE-RT"; \
		export ETROBO_ENV_TARGET="physical"; \
		export ETROBO_SPIKE_RT_ROOT="$$_SPIKE_RT_ROOT"; \
		export ETROBO_TARGET_SDK="$$_SPIKE_RT_ROOT/sdk"; \
		export ETROBO_TARGET_WORKSPACE="$$_SPIKE_RT_ROOT/sdk/workspace"; \
		export ETROBO_SPIKE_RT_TOOLS="$$_SPIKE_RT_ROOT/tools"; \
	fi; \
	if [ -z "$$ETROBO_SPIKE_RT_TOOLS" ]; then \
		export ETROBO_SPIKE_RT_TOOLS="$$_SPIKE_RT_ROOT/tools"; \
	fi; \
	if [ -z "$$ETROBO_SPIKE_RT_ROOT" ]; then \
		export ETROBO_SPIKE_RT_ROOT="$$_SPIKE_RT_ROOT"; \
	fi; \
	_TOOLCHAIN_DIR="$${ETROBO_TARGET_GCC:-$$_ETROBO_ROOT/gcc-arm-none-eabi-10.3-2021.10}/bin"; \
	if [ ! -d "$$_TOOLCHAIN_DIR" ]; then \
		_TOOLCHAIN_DIR="$$_SPIKE_RT_ROOT/gcc-arm-none-eabi-10.3-2021.10/bin"; \
	fi; \
	if [ -d "$$_TOOLCHAIN_DIR" ]; then export PATH="$$_TOOLCHAIN_DIR:$$PATH"; fi; \
	if [ -z "$$PYTHON3" ]; then \
		if [ -x "$$ETROBO_SPIKE_RT_TOOLS/python/bin/python3" ]; then \
			export PYTHON3="$$ETROBO_SPIKE_RT_TOOLS/python/bin/python3"; \
		else \
			export PYTHON3=python3; \
		fi; \
	fi; \
	unset _APP_DIR _APP_REAL_DIR _SPIKE_RT_ROOT _ETROBO_ROOT _TOOLCHAIN_DIR;

.DEFAULT_GOAL := build

.PHONY: help build upload up flash device clean realclean rebuild

help:
	@printf '%s\n' \
		'Usage from spike-rt/sdk/workspace/$(APP):' \
		'  make            Build $(APP)' \
		'  make up         Build and upload $(APP) to a Hub in DFU mode' \
		'  make upload     Upload the last built asp.bin' \
		'  make device     Show connected SPIKE Hub mode' \
		'  make clean      Clean the current build' \
		'  make realclean  Clean build and kernel artifacts'

build:
	@bash -lc '\
		$(APP_ENV) \
		set -eo pipefail; \
		if ! command -v arm-none-eabi-g++ >/dev/null 2>&1; then \
			echo "[$(APP)] arm-none-eabi-g++ was not found. Install gcc-arm-none-eabi-10.3-2021.10 or add it to PATH."; \
			exit 1; \
		fi; \
		if [ ! -f "$$ETROBO_SPIKE_RT_ROOT/external/libpybricks/micropython/tools/dfu.py" ]; then \
			echo "[$(APP)] external/libpybricks is missing. Run: git submodule update --init --recursive"; \
			exit 1; \
		fi; \
		echo "[$(APP)] build"; \
		$(MAKE) -C "$$ETROBO_TARGET_WORKSPACE" img="$(APP)" \
	'

upload:
	@bash -lc '\
		$(APP_ENV) \
		set -eo pipefail; \
		cd "$$ETROBO_TARGET_WORKSPACE"; \
			if [ ! -f asp.bin ]; then \
				echo "[$(APP)] asp.bin was not found. Run make or make up first."; \
				exit 1; \
			fi; \
			mode=""; \
			if command -v spike >/dev/null 2>&1; then \
				mode="$$(spike device 2>/dev/null || true)"; \
			fi; \
			if [ -n "$$mode" ] && [ "$$mode" != "DFU" ]; then \
				echo "[$(APP)] SPIKE Hub is not in DFU mode. Current mode: $${mode:-not found}"; \
				echo "[$(APP)] Hold the Bluetooth button while plugging in USB, then run make upload again."; \
				exit 1; \
			elif [ -z "$$mode" ]; then \
				echo "[$(APP)] skip DFU pre-check because spike command is not available."; \
			fi; \
			echo "[$(APP)] upload asp.bin"; \
			set +e; \
		sudo ETROBO_SPIKE_RT_TOOLS="$$ETROBO_SPIKE_RT_TOOLS" PYTHON3="$$PYTHON3" "$(COMMON_DIR)/deploy-dfu.sh" asp.bin; \
		rc=$$?; \
		set -e; \
		if [ $$rc -ne 0 ]; then \
			echo; \
			echo "[$(APP)] upload failed (exit code: $$rc)"; \
			if [ $$rc -eq 26 ]; then \
				echo "[$(APP)] Exit code 26 usually points to the DFU/USB upload step."; \
			fi; \
			echo "[$(APP)] Check DFU mode, USB cable, direct PC connection, and libusb/udev/usbipd permissions."; \
			exit $$rc; \
		fi; \
		echo "asp.bin has been successfully uploaded." \
	'

up: build upload

flash: up

device:
	@bash -lc '\
		$(APP_ENV) \
		set -eo pipefail; \
		if ! command -v spike >/dev/null 2>&1; then \
			echo "spike command was not found"; \
			exit 1; \
		fi; \
		mode="$$(spike device 2>/dev/null || true)"; \
		if [ -n "$$mode" ]; then \
			echo "$$mode"; \
		else \
			echo "not found"; \
			exit 1; \
		fi \
	'

clean:
	@bash -lc '\
		$(APP_ENV) \
		set -eo pipefail; \
		$(MAKE) -C "$$ETROBO_TARGET_WORKSPACE" clean \
	'

realclean:
	@bash -lc '\
		$(APP_ENV) \
		set -eo pipefail; \
		$(MAKE) -C "$$ETROBO_TARGET_WORKSPACE" realclean \
	'

rebuild: clean build
