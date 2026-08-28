APP ?= etrobo_nao
APP_DIR := sdk/workspace/$(APP)

.DEFAULT_GOAL := build

.PHONY: help setup prepare-submodules setup-upload build upload up flash device doctor check clean realclean rebuild

help:
	@printf '%s\n' \
		'Usage from the repository root:' \
		'  make setup       Prepare required submodules' \
		'  make setup-upload  Prepare Python USB upload deps' \
		'  make             Build $(APP)' \
		'  make up          Build and upload $(APP) to a Hub in DFU mode' \
		'  make upload      Upload the last built asp.bin' \
		'  make device      Show connected SPIKE Hub mode' \
		'  make doctor      Check local build/upload environment' \
		'  make clean       Clean the current build' \
		'  make realclean   Clean build and kernel artifacts' \
		'  make APP=<name>  Use another app under sdk/workspace/'

setup:
	@$(MAKE) -C "$(APP_DIR)" setup

prepare-submodules:
	@$(MAKE) -C "$(APP_DIR)" prepare-submodules

setup-upload:
	@$(MAKE) -C "$(APP_DIR)" setup-upload

build:
	@$(MAKE) -C "$(APP_DIR)" build

upload:
	@$(MAKE) -C "$(APP_DIR)" upload

up:
	@$(MAKE) -C "$(APP_DIR)" up

flash:
	@$(MAKE) -C "$(APP_DIR)" flash

device:
	@$(MAKE) -C "$(APP_DIR)" device

doctor:
	@$(MAKE) -C "$(APP_DIR)" doctor

check: doctor

clean:
	@$(MAKE) -C "$(APP_DIR)" clean

realclean:
	@$(MAKE) -C "$(APP_DIR)" realclean

rebuild:
	@$(MAKE) -C "$(APP_DIR)" rebuild
