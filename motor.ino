#define ENC_A1 3
#define ENC_A2 5
#define ENC_B1 2
#define ENC_B2 4

#define MOT_A1 8
#define MOT_A2 9
#define MOT_APWM 10
#define MOT_B1 13
#define MOT_B2 12
#define MOT_BPWM 11

#define SPEED_STEPS 255

#define BAUD 9600

// describes an encoder
struct encoder {
	int count;
	int pin1, pin2;
};

// describes a motor
struct motor {
	int pin1, pin2, pinpwm;
};

// Initialize an encoder struct.
void encoder_init(volatile struct encoder *encoder, int pin1, int pin2);

// If there is a signal rise, update the encoder count.
void encoder_count(volatile struct encoder *encoder);

// interrupt handlers
void handler_a();
void handler_b();

// Print state information for testing.
void print_state();

// Initialize a motor struct.
void motor_init(struct motor *motor, int pin1, int pin2, int pinpwm);

// Run a motor at speed, where -1.0 <= speed <= 1.0.
void motor_run(struct motor *motor, float speed);

// Brake a motor with strength, where -1.0 <= strength <= 1.0.
void motor_brake(struct motor *motor, float strength);

// Cause a motor to coast.
void motor_coast(struct motor *motor);


volatile struct encoder encoder_a, encoder_b;
struct motor motor_a, motor_b;
int handler_a_isset = 0;


void setup()
{
	encoder_init(&encoder_a, ENC_A1, ENC_A2);
	encoder_init(&encoder_b, ENC_B1, ENC_B2);

	motor_init(&motor_a, MOT_A1, MOT_A2, MOT_APWM);
	motor_init(&motor_b, MOT_B1, MOT_B2, MOT_BPWM);

	Serial.begin(BAUD);
}

void loop()
{
	// Example code: Run each motor back and forth by about one shaft turn.

	motor_run(&motor_a, 0.10);

	while (encoder_a.count < 22) {
		delay(10);
	}

	motor_brake(&motor_a, 1.0);

	delay(1000);

	motor_run(&motor_a, -0.10);

	while (encoder_a.count > 0) {
		delay(10);
	}

	motor_brake(&motor_a, 1.0);

	delay(1000);


	motor_run(&motor_b, 0.10);

	while (encoder_b.count < 22)
		delay(10);

	motor_brake(&motor_b, 1.0);

	delay(1000);

	motor_run(&motor_b, -0.10);

	while (encoder_b.count > 0)
		delay(10);

	motor_brake(&motor_b, 1.0);

	delay(1000);
}

void encoder_init(volatile struct encoder *encoder, int pin1, int pin2)
{
	int NOT_AN_INTERRUPT = 6969; // Wouldn't compile without this line for
	                             // some reason. Need to look into this.

	encoder->count = 0;
	encoder->pin1 = pin1;
	encoder->pin2 = pin2;

	pinMode(pin1, INPUT);
	pinMode(pin2, INPUT);

	if (!handler_a_isset) {
		attachInterrupt(digitalPinToInterrupt(pin1), handler_a, RISING);
		handler_a_isset = 1;
	}
	else {
		attachInterrupt(digitalPinToInterrupt(pin1), handler_b, RISING);
	}
}

void encoder_count(volatile struct encoder *encoder)
{
	int pin2_state;

	pin2_state = digitalRead(encoder->pin2);

	if (pin2_state == LOW)
		encoder->count++;
	else
		encoder->count--;
}

void handler_a()
{
	encoder_count(&encoder_a);
}

void handler_b()
{
	encoder_count(&encoder_b);
}

void print_state()
{
	Serial.print("Encoder A: ");
	Serial.print(encoder_a.count);
	Serial.print("   ");
	Serial.print("Encoder B: ");
	Serial.print(encoder_b.count);
	Serial.println();
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
