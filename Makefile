ARDUINO_PATH = /opt/teensyduino
SKETCHBOOK   = $(HOME)/sketchbook
SKETCH       = $(notdir $(CURDIR)).ino
TARGET_DIR   = /home/aaron/build-teensy
MONITOR_PORT = /dev/ttyUSB0

all:
	@ mkdir -p $(TARGET_DIR)

	$(ARDUINO_PATH)/arduino-builder -compile -logger=machine \
	-hardware "$(ARDUINO_PATH)/hardware" \
	-tools "$(ARDUINO_PATH)/tools-builder" \
	-tools "$(ARDUINO_PATH)/hardware/tools/avr" \
	-built-in-libraries "$(ARDUINO_PATH)/libraries" \
	-fqbn=teensy:avr:teensy31:usb=serial,speed=144opt,keys=en-us \
	-ide-version=10606 \
	-build-path "$(TARGET_DIR)" \
	"$(SKETCH)"

upload: all flash

clean:
	rm -rf $(TARGET_DIR)

monitor:
	screen $(MONITOR_PORT) 9600

