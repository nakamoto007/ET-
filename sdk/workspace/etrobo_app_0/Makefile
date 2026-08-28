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
	_TOOLCHAIN_BASE="$${ETROBO_TARGET_GCC:-}"; \
	_TOOLCHAIN_DIR=""; \
	if [ -n "$$_TOOLCHAIN_BASE" ]; then \
		if [ -d "$$_TOOLCHAIN_BASE/bin" ]; then \
			_TOOLCHAIN_DIR="$$_TOOLCHAIN_BASE/bin"; \
		elif [ -d "$$_TOOLCHAIN_BASE" ]; then \
			_TOOLCHAIN_DIR="$$_TOOLCHAIN_BASE"; \
		fi; \
	fi; \
	if [ -z "$$_TOOLCHAIN_DIR" ]; then \
		for _CANDIDATE in "$$_ETROBO_ROOT/gcc-arm-none-eabi-10.3-2021.10/bin" "$$_SPIKE_RT_ROOT/gcc-arm-none-eabi-10.3-2021.10/bin"; do \
			if [ -d "$$_CANDIDATE" ]; then \
				_TOOLCHAIN_DIR="$$_CANDIDATE"; \
				break; \
			fi; \
		done; \
	fi; \
	if [ -n "$$_TOOLCHAIN_DIR" ]; then export PATH="$$_TOOLCHAIN_DIR:$$PATH"; fi; \
	if [ -z "$$PYTHON3" ]; then \
		if [ -x "$$ETROBO_SPIKE_RT_TOOLS/python/bin/python3" ]; then \
			export PYTHON3="$$ETROBO_SPIKE_RT_TOOLS/python/bin/python3"; \
		else \
			export PYTHON3=python3; \
		fi; \
	fi; \
	unset _APP_DIR _APP_REAL_DIR _SPIKE_RT_ROOT _ETROBO_ROOT _TOOLCHAIN_BASE _TOOLCHAIN_DIR _CANDIDATE;

.DEFAULT_GOAL := build

.PHONY: help setup prepare-submodules setup-upload build upload up flash device doctor check clean realclean rebuild

help:
	@printf '%s\n' \
		'Usage from spike-rt/sdk/workspace/$(APP):' \
		'  make setup      Prepare required submodules' \
		'  make setup-upload  Prepare Python USB upload deps' \
		'  make            Build $(APP)' \
		'  make up         Build and upload $(APP) to a Hub in DFU mode' \
		'  make upload     Upload the last built asp.bin' \
		'  make device     Show connected SPIKE Hub mode' \
		'  make doctor     Check local build/upload environment' \
		'  make clean      Clean the current build' \
		'  make realclean  Clean build and kernel artifacts'

setup: prepare-submodules

prepare-submodules:
	@bash -lc '\
		$(APP_ENV) \
		set -eo pipefail; \
		cd "$$ETROBO_SPIKE_RT_ROOT"; \
		echo "[$(APP)] prepare submodules"; \
		git submodule update --init drivers/tlsf external/libpybricks; \
		git -C external/libpybricks submodule update --init micropython; \
		git -C external/libpybricks submodule update --init --checkout lib/btstack; \
		git -C external/libpybricks/micropython submodule update --init lib/stm32lib; \
		git -C external/libpybricks/micropython submodule deinit -f lib/btstack >/dev/null 2>&1 || true; \
		echo "[$(APP)] submodules ready" \
	'

setup-upload:
	@bash -lc '\
		$(APP_ENV) \
		set -eo pipefail; \
		cd "$$ETROBO_SPIKE_RT_ROOT"; \
		if ! command -v python3 >/dev/null 2>&1; then \
			echo "[$(APP)] python3 was not found."; \
			exit 1; \
		fi; \
		if [ ! -x tools/python/bin/python3 ]; then \
			python3 -m venv tools/python; \
		fi; \
		tools/python/bin/pip install -r tools/requirements.txt; \
		echo "[$(APP)] Python USB upload deps ready" \
	'

build:
	@bash -lc '\
		$(APP_ENV) \
		set -eo pipefail; \
		if ! command -v arm-none-eabi-g++ >/dev/null 2>&1; then \
			echo "[$(APP)] arm-none-eabi-g++ was not found."; \
			echo "[$(APP)] Put gcc-arm-none-eabi-10.3-2021.10 next to the repo, inside the repo, add it to PATH, or run:"; \
			echo "[$(APP)]   ETROBO_TARGET_GCC=/actual/path/gcc-arm-none-eabi-10.3-2021.10 make"; \
			exit 1; \
		fi; \
		if [ ! -f "$$ETROBO_SPIKE_RT_ROOT/external/libpybricks/micropython/tools/dfu.py" ]; then \
			echo "[$(APP)] external/libpybricks is missing. Run: make setup"; \
			exit 1; \
		fi; \
		if [ ! -f "$$ETROBO_SPIKE_RT_ROOT/external/libpybricks/lib/btstack/tool/compile_gatt.py" ]; then \
			echo "[$(APP)] external/libpybricks/lib/btstack is missing. Run: make setup"; \
			exit 1; \
		fi; \
		if [ ! -f "$$ETROBO_SPIKE_RT_ROOT/external/libpybricks/micropython/lib/stm32lib/README.md" ]; then \
			echo "[$(APP)] external/libpybricks/micropython/lib/stm32lib is missing. Run: make setup"; \
			exit 1; \
		fi; \
		if [ -f "$$ETROBO_SPIKE_RT_ROOT/external/libpybricks/micropython/lib/btstack/src/btstack.h" ]; then \
			echo "[$(APP)] external/libpybricks/micropython/lib/btstack must stay deinitialized for this SPIKE-RT build."; \
			echo "[$(APP)] Run: make setup"; \
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

doctor:
	@bash -lc '\
		$(APP_ENV) \
		set +e; \
		rc=0; \
		echo "[$(APP)] environment check"; \
		echo "app: $(APP_DIR)"; \
		echo "workspace: $$ETROBO_TARGET_WORKSPACE"; \
		if command -v arm-none-eabi-g++ >/dev/null 2>&1; then \
			echo "ok: arm-none-eabi-g++ -> $$(command -v arm-none-eabi-g++)"; \
		else \
			echo "ng: arm-none-eabi-g++ was not found"; \
			echo "    Put gcc-arm-none-eabi-10.3-2021.10 next to the repo, inside the repo, add it to PATH, or set ETROBO_TARGET_GCC."; \
			rc=1; \
		fi; \
		if [ -f "$$ETROBO_SPIKE_RT_ROOT/external/libpybricks/micropython/tools/dfu.py" ]; then \
			echo "ok: external/libpybricks/micropython"; \
		else \
			echo "ng: external/libpybricks/micropython is missing"; \
			echo "    Run: make setup"; \
			rc=1; \
		fi; \
		if [ -f "$$ETROBO_SPIKE_RT_ROOT/external/libpybricks/lib/btstack/tool/compile_gatt.py" ]; then \
			echo "ok: external/libpybricks/lib/btstack"; \
		else \
			echo "ng: external/libpybricks/lib/btstack is missing"; \
			echo "    Run: make setup"; \
			rc=1; \
		fi; \
		if [ -f "$$ETROBO_SPIKE_RT_ROOT/external/libpybricks/micropython/lib/stm32lib/README.md" ]; then \
			echo "ok: external/libpybricks/micropython/lib/stm32lib"; \
		else \
			echo "ng: external/libpybricks/micropython/lib/stm32lib is missing"; \
			echo "    Run: make setup"; \
			rc=1; \
		fi; \
		if [ -f "$$ETROBO_SPIKE_RT_ROOT/external/libpybricks/micropython/lib/btstack/src/btstack.h" ]; then \
			echo "ng: external/libpybricks/micropython/lib/btstack is initialized"; \
			echo "    Run: make setup"; \
			echo "    Do not use git submodule update --init --recursive for this repo."; \
			rc=1; \
		else \
			echo "ok: external/libpybricks/micropython/lib/btstack is deinitialized"; \
		fi; \
		if [ -d "$$ETROBO_SPIKE_RT_ROOT/drivers/tlsf" ]; then \
			echo "ok: drivers/tlsf submodule"; \
		else \
			echo "ng: drivers/tlsf submodule is missing"; \
			echo "    Run: make setup"; \
			rc=1; \
		fi; \
		if command -v "$$PYTHON3" >/dev/null 2>&1; then \
			echo "ok: python3 -> $$PYTHON3"; \
			"$$PYTHON3" -c "import usb" >/dev/null 2>&1; \
			if [ $$? -eq 0 ]; then \
				echo "ok: pyusb"; \
			else \
				echo "ng: pyusb is not installed for $$PYTHON3"; \
				echo "    Run: make setup-upload"; \
				rc=1; \
			fi; \
		else \
			echo "ng: python3 was not found"; \
			rc=1; \
		fi; \
		if command -v spike >/dev/null 2>&1; then \
			echo "ok: spike -> $$(command -v spike)"; \
		else \
			echo "warn: spike command was not found; make device will be unavailable."; \
		fi; \
		exit $$rc \
	'

check: doctor

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
