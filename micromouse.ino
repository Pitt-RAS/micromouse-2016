#include "conf.h"

#include "motors.h"

#define BAUD 9600

void setup()
{
	Serial2.begin(BAUD);
    motors_init();
}

void loop()
{
    if (abs(knobLeft.read() - 1000) <= 10) {
        motor_brake(&motor_a, 1.0);
    } else {
        motor_run(&motor_a, constrain(-kP * knobLeft.read(), -1, 1));
    }
    if (abs(knobRight.read() - 1000) <= 10) {
        motor_brake(&motor_b, 1.0);
    } else {
        motor_run(&motor_b, constrain(-kP * knobRight.read(), -1, 1));
    }
    Serial2.print("Left: ");
    Serial2.print(knobLeft.read());
    Serial2.print("\tRight: ");
    Serial2.println(knobRight.read());
	// Example code: Run both motors back and forth

//	motor_run(&motor_a, 0.15);
//  motor_run(&motor_b, 0.15);
//
//	while (knobLeft.read() < 50 && knobRight.read() < 50) {
//    Serial2.print("Left: ");
//    Serial2.print(knobLeft.read());
//    Serial2.print("\tRight: ");
//    Serial2.println(knobRight.read());
//		delay(10);
//	}
//  if (knobLeft.read() >= 50) {
//    motor_brake(&motor_a, 1.0);
//  }
//  if (knobRight.read() >= 50) {
//    motor_brake(&motor_b, 1.0);
//  }
//  while (knobLeft.read() < 50 || knobRight.read() < 50) {
//    Serial2.print("Left: ");
//    Serial2.print(knobLeft.read());
//    Serial2.print("\tRight: ");
//    Serial2.println(knobRight.read());
//    delay(10);
//  }
//
//	motor_brake(&motor_a, 1.0);
//  motor_brake(&motor_b, 1.0);
//
//	delay(1000);
//
//  motor_run(&motor_a, -0.15);
//	motor_run(&motor_b, -0.15);
//
//	while (knobLeft.read() > 0 && knobRight.read() > 0) {
//    Serial2.print("Left: ");
//    Serial2.print(knobLeft.read());
//    Serial2.print("\tRight: ");
//    Serial2.println(knobRight.read());
//    delay(10);
//  }
//  if (knobLeft.read() <= 0) {
//    motor_brake(&motor_a, 1.0);
//  }
//  if (knobRight.read() <= 0) {
//    motor_brake(&motor_b, 1.0);
//  }
//  while (knobLeft.read() > 0 || knobRight.read() > 0) {
//    Serial2.print("Left: ");
//    Serial2.print(knobLeft.read());
//    Serial2.print("\tRight: ");
//    Serial2.println(knobRight.read());
//    delay(10);
//  }
//
//  motor_brake(&motor_a, 1.0);
//	motor_brake(&motor_b, 1.0);
//
//	delay(1000);
}

