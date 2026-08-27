APP_DIR := $(abspath $(dir $(lastword $(MAKEFILE_LIST))))
APP := $(notdir $(patsubst %/,%,$(APP_DIR)))
WORKSPACE := $(abspath $(APP_DIR)/..)
COMMON_DIR := $(abspath $(WORKSPACE)/../common)
APP_ENV := _APP_DIR="$(APP_DIR)"; \
	_APP_REAL_DIR="$$(cd "$$_APP_DIR" && pwd -P)"; \
	export ETROBO_ROOT="$$(cd "$$_APP_REAL_DIR/../../../.." && pwd -P)"; \
	source "$$ETROBO_ROOT/scripts/etroboenv.sh" >/dev/null; \
	_TOOLCHAIN_DIR="$$ETROBO_ROOT/gcc-arm-none-eabi-10.3-2021.10/bin"; \
	if [ -d "$$_TOOLCHAIN_DIR" ]; then export PATH="$$_TOOLCHAIN_DIR:$$PATH"; fi; \
	unset _APP_DIR _APP_REAL_DIR _TOOLCHAIN_DIR;

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
		mode="$$(spike device 2>/dev/null || true)"; \
		if [ "$$mode" != "DFU" ]; then \
			echo "[$(APP)] SPIKE Hub is not in DFU mode. Current mode: $${mode:-not found}"; \
			echo "[$(APP)] Hold the Bluetooth button while plugging in USB, then run make upload again."; \
			exit 1; \
		fi; \
		echo "[$(APP)] upload asp.bin"; \
		set +e; \
		sudo ETROBO_SPIKE_RT_TOOLS="$$ETROBO_SPIKE_RT_TOOLS" "$(COMMON_DIR)/deploy-dfu.sh" asp.bin; \
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
