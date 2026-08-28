mkfile_path := $(dir $(lastword $(MAKEFILE_LIST)))

SRCLANG = c++

APPL_CXXOBJS +=\
AllSensors.o\
BluetoothSender.o\
BottleDetection.o\
challenges.o\
ColorDetector.o\
ColorSensorService.o\
DriveBase.o\
DriveController.o\
LineTracer.o\
LineTraceController.o\
RobotSensors.o\
RobotController.o\
RobotStateController.o\
CompetitionScenario.o\
CompetitionSections.o\
SensorCsvLogger.o\
SensorLiftController.o\
UltrasonicSensor.o

APPL_COBJS +=\
HubIMU.o

APPL_LIBS += -lm

APPL_DIRS += $(mkfile_path)config
APPL_DIRS += $(mkfile_path)control
APPL_DIRS += $(mkfile_path)drive
APPL_DIRS += $(mkfile_path)HubIMU
APPL_DIRS += $(mkfile_path)LineTracer
APPL_DIRS += $(mkfile_path)bottle_detection
APPL_DIRS += $(mkfile_path)challenges
APPL_DIRS += $(mkfile_path)logging
APPL_DIRS += $(mkfile_path)scenario
APPL_DIRS += $(mkfile_path)sections
APPL_DIRS += $(mkfile_path)sensors

INCLUDES += \
	-I$(mkfile_path)bottle_detection\
	-I$(mkfile_path)challenges\
	-I$(mkfile_path)config\
	-I$(mkfile_path)control\
	-I$(mkfile_path)drive\
	-I$(mkfile_path)HubIMU\
	-I$(mkfile_path)LineTracer\
	-I$(mkfile_path)logging\
	-I$(mkfile_path)scenario\
	-I$(mkfile_path)sections\
	-I$(mkfile_path)sensors
