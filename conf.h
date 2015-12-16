#ifndef CONF_H
#define CONF_H

// Define pin assignments for teensy3.2 on Rev1 PCB
#define ENCODER_A1 1
#define ENCODER_A2 0
#define MOTOR_A1 4
#define MOTOR_A2 2
#define MOTOR_AP 3

#define ENCODER_B1 12
#define ENCODER_B2 11
#define MOTOR_B1 7
#define MOTOR_B2 6
#define MOTOR_BP 5

#define BUTTON1 23
#define BUTTON2 22
#define BUTTON3 21

#define SCL_ 18
#define SDA_ 19

#define EMITTER1 29
#define EMITTER2 30
#define EMITTER3 31
#define EMITTER4 32
#define EMITTER5 33

#define RANGE1 A14
#define RANGE2 A0
#define RANGE3 A1
#define RANGE4 A2
#define RANGE5 A3

// Range sensor directions
#define RANGE_FRONT_LEFT RANGE1
#define RANGE_FRONT_RIGHT RANGE2
#define RANGE_LEFT RANGE3
#define RANGE_RIGHT RANGE4
#define RANGE_FRONT RANGE5

// Range sensor distance offsets
#define RANGE_FRONT_LEFT_OFFSET 0
#define RANGE_FRONT_RIGHT_OFFSET 0
#define RANGE_LEFT_OFFSET 0
#define RANGE_RIGHT_OFFSET 0
#define RANGE_FRONT_OFFSET 0

// Motion control paremeters
#define MM_PER_BLOCK 180
#define MM_PER_STEP 0.7928

// PID tuning parameters
#define KP 0.4
#define KD 0
#define KI 0

// Robot characteristics
#define ROBOT_MASS .226 // kilograms
#define MM_BETWEEN_WHEELS 75 // millimeters
#define NUMBER_OF_MOTORS 2
#define STEPS_PER_MOTOR_REV 12// the number of encoder steps we get per wheel revolution
#define BATTERY_VOLTAGE 8.1 // Volts
#define MAX_ACCEL 4 // m/s/s  
#define MAX_DECEL -4 // m/s/s
#define GEAR_RATIO 9.96 // gear ratio between motor and wheels
#define MAX_VELOCITY_STRAIGHT 2 // m/s   limited by the maximum velocity at which motors can deliver max accel
#define PWM_SPEED_STEPS 1023 // maximum PWM value for the system

// Motor spec sheet parameters
#define RATED_VOLTAGE 8 // Voltage that ratings were measured with
#define RATED_INTERNAL_RESISTANCE 3.1 // ohms of resistance in motor coils
#define RATED_FREERUN_CURRENT 0.06// Amps
#define RATED_RPM_PER_VBEMF 818000 // RPM/Volt
#define RATED_TORQUE_PER_AMP 0.000622353 // torque in N-m at output shaft, Amps passed through motor

// Convert motor parameters to robot parameters
#define WHEEL_RADIUS (MM_PER_STEP * STEPS_PER_MOTOR_REV * GEAR_RATIO / (2 * 3.14159265359))// in millimeters
#define FORCE_PER_AMP (RATED_TORQUE_PER_AMP * GEAR_RATIO * NUMBER_OF_MOTORS * 1000 / WHEEL_RADIUS) // in Newtons
#define VELOCITY_PER_VBEMF (RATED_RPM_PER_VBEMF * STEPS_PER_MOTOR_REV * GEAR_RATIO * MM_PER_STEP / (60000)) // 60000 is for mm to m and s to min
#define FRICTION_FORCE (RATED_FREERUN_CURRENT * FORCE_PER_AMP + 0.6) // Newtons (0.08 calculated Newtons from motor/gearbox)  amount of force opposing motion in robot including rolling resistance, sliding, gearing

//  TODO precompile, calculate max velocity based on turn radius and max accel, which will then limit max velocity through centripital force.  
//    if this max velocity is higher than max straight velocity then use max straight velocity
#define MAX_VELOCITY_ROTATE .05 // m/s





#endif
