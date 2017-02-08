#include "../device/RangeSensorContainer.h"
#include "../user_interaction/UserInterface.h"
#include "stream_ranges.h"

void streamRanges() {
  while (!gUserInterface.buttonBackPressed()) {
    RangeSensors.updateReadings();
    Serial.print(RangeSensors.diagLeftSensor.getRange());
    Serial.print("\t");
    Serial.print(RangeSensors.diagRightSensor.getRange());
    Serial.print("\t");
    Serial.print(RangeSensors.frontLeftSensor.getRange());
    Serial.print("\t");
    Serial.print(RangeSensors.frontRightSensor.getRange());
    Serial.println();
  }
}

void streamRawRanges() {
  while (!gUserInterface.buttonBackPressed()) {
    RangeSensors.updateReadings();
    Serial.print(RangeSensors.diagLeftSensor.getRawReading());
    Serial.print("\t");
    Serial.print(RangeSensors.diagRightSensor.getRawReading());
    Serial.print("\t");
    Serial.print(RangeSensors.frontLeftSensor.getRawReading());
    Serial.print("\t");
    Serial.print(RangeSensors.frontRightSensor.getRawReading());
    Serial.println();
  }
}
