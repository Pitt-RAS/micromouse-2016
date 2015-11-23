// describes a motor
struct motor {
    int pin1, pin2, pinpwm;
} motor_a, motor_b;

// Initialize motors
void motors_init();

// Run a motor at speed, where -1.0 <= speed <= 1.0.
void motor_run(struct motor *motor, float speed);

// Brake a motor with strength, where -1.0 <= strength <= 1.0.
void motor_brake(struct motor *motor, float strength);

// Cause a motor to coast.
void motor_coast(struct motor *motor);
