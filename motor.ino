#include "conf.h"
#include <Encoder.h>

#define SPEED_STEPS 255

#define BAUD 9600

// describes a motor
struct motor {
	int pin1, pin2, pinpwm;
};

// Initialize a motor struct.
void motor_init(struct motor *motor, int pin1, int pin2, int pinpwm);

// Run a motor at speed, where -1.0 <= speed <= 1.0.
void motor_run(struct motor *motor, float speed);

// Brake a motor with strength, where -1.0 <= strength <= 1.0.
void motor_brake(struct motor *motor, float strength);

// Cause a motor to coast.
void motor_coast(struct motor *motor);

struct motor motor_a, motor_b;

Encoder knobLeft(ENCODER_A1, ENCODER_A2);
Encoder knobRight(ENCODER_B1, ENCODER_B2);

void setup()
{
	motor_init(&motor_a, MOTOR_A1, MOTOR_A2, MOTOR_AP);
	motor_init(&motor_b, MOTOR_B1, MOTOR_B2, MOTOR_BP);

	Serial2.begin(BAUD);
}

long positionLeft  = -999;
long positionRight = -999;

float kP = 0.01;

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

void motor_init(struct motor *motor, int pin1, int pin2, int pinpwm)
{
	motor->pin1 = pin1;
	motor->pin2 = pin2;
	motor->pinpwm = pinpwm;

	pinMode(pin1, OUTPUT);
	pinMode(pin2, OUTPUT);
	pinMode(pinpwm, OUTPUT);
}

void motor_run(struct motor *motor, float speed)
{
	int pin1_state, pin2_state;
	int speed_raw;

	if (speed < -1.0 || speed > 1.0)
		speed = 0.0;

	speed_raw = (int)(SPEED_STEPS * speed);

	if (speed_raw < 0)
		speed_raw *= -1;

	pin1_state = LOW;
	pin2_state = LOW;

	if (speed > 0.0)
		pin1_state = HIGH;
	else if (speed < 0.0)
		pin2_state = HIGH;

	digitalWrite(motor->pin1, pin1_state);
	digitalWrite(motor->pin2, pin2_state);
	analogWrite(motor->pinpwm, speed_raw);
}

void motor_brake(struct motor *motor, float strength)
{
	int strength_raw;

	if (strength < 0.0 || strength > 1.0)
		strength = 0.0;

	strength_raw = (int)(SPEED_STEPS * strength);

	digitalWrite(motor->pin1, HIGH);
	digitalWrite(motor->pin2, HIGH);
	analogWrite(motor->pinpwm, strength_raw);
}

void motor_coast(struct motor *motor)
{
	digitalWrite(motor->pin1, LOW);
	digitalWrite(motor->pin2, LOW);
	analogWrite(motor->pinpwm, 0);
}
