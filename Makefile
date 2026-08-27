APP ?= etrobo_app_0
APP_DIR := sdk/workspace/$(APP)

.DEFAULT_GOAL := build

.PHONY: help build upload up flash device clean realclean rebuild

help:
	@printf '%s\n' \
		'Usage from the repository root:' \
		'  make             Build $(APP)' \
		'  make up          Build and upload $(APP) to a Hub in DFU mode' \
		'  make upload      Upload the last built asp.bin' \
		'  make device      Show connected SPIKE Hub mode' \
		'  make clean       Clean the current build' \
		'  make realclean   Clean build and kernel artifacts' \
		'  make APP=<name>  Use another app under sdk/workspace/'

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

clean:
	@$(MAKE) -C "$(APP_DIR)" clean

realclean:
	@$(MAKE) -C "$(APP_DIR)" realclean

rebuild:
	@$(MAKE) -C "$(APP_DIR)" rebuild
