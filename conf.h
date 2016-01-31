#ifndef CONF_H
#define CONF_H

#include <Arduino.h>

// Define pin assignments for teensy3.2 on Rev1 PCB
#define ENCODER_A1_PIN 1
#define ENCODER_A2_PIN 0
#define MOTOR_A1_PIN 4
#define MOTOR_A2_PIN 2
#define MOTOR_AP_PIN 3

#define ENCODER_B1_PIN 12
#define ENCODER_B2_PIN 11
#define MOTOR_B1_PIN 7
#define MOTOR_B2_PIN 6
#define MOTOR_BP_PIN 5

#define BUTTON1_PIN 23
#define BUTTON2_PIN 22
#define BUTTON3_PIN 21

#define SCL_PIN 18
#define SDA_PIN 19

#define EMITTER1_PIN 29
#define EMITTER2_PIN 30
#define EMITTER3_PIN 31
#define EMITTER4_PIN 32
#define EMITTER5_PIN 33

#define RANGE1_PIN A14
#define RANGE2_PIN A0
#define RANGE3_PIN A1
#define RANGE4_PIN A2
#define RANGE5_PIN A3

// RANGE SENSOR OPTIONS

// Range sensor directions
#define RANGE_DIAG_LEFT_PIN RANGE1_PIN
#define RANGE_DIAG_RIGHT_PIN RANGE2_PIN
#define RANGE_LEFT_PIN RANGE3_PIN
#define RANGE_RIGHT_PIN RANGE4_PIN
#define RANGE_FRONT_PIN RANGE5_PIN

// Range sensor distance offsets
#define RANGE_DIAG_LEFT_OFFSET -35
#define RANGE_DIAG_RIGHT_OFFSET -35
#define RANGE_LEFT_OFFSET 15
#define RANGE_RIGHT_OFFSET 15
#define RANGE_FRONT_OFFSET 30

// Range sensor wall thresholds
#define RANGE_DIAG_LEFT_WALL_THRESHOLD 75
#define RANGE_DIAG_RIGHT_WALL_THRESHOLD 75
#define RANGE_LEFT_WALL_THRESHOLD 70
#define RANGE_RIGHT_WALL_THRESHOLD 70
#define RANGE_FRONT_WALL_THRESHOLD 125

// Range sensor middle readings
#define RANGE_DIAG_LEFT_MIDDLE 80
#define RANGE_DIAG_RIGHT_MIDDLE 80
#define RANGE_LEFT_MIDDLE 100
#define RANGE_RIGHT_MIDDLE 100
#define RANGE_FRONT_MIDDLE 100

// Number of samples in moving average
#define RANGE_QUEUE_MAX_LENGTH 10
#define HISTORY_QUEUE_MAX_LENGTH 100

// Number of old samples to clear when wall status changes
#define RANGE_QUEUE_NUM_TO_CLEAR 5

// Motion control paremeters
#define MM_PER_BLOCK 180
#define MM_PER_STEP 0.7928

// PID tuning parameters
#define KP_POSITION .01
#define KI_POSITION 0
#define KD_POSITION 0

#define KP_ROTATION 1
#define KI_ROTATION 0
#define KD_ROTATION 0

// Robot characteristics
#define ROBOT_MASS .226 // kilograms
#define MM_BETWEEN_WHEELS 76.75 // millimeters
#define NUMBER_OF_MOTORS 2
#define STEPS_PER_MOTOR_REV 12// the number of encoder steps we get per wheel revolution
#define BATTERY_VOLTAGE 8.1 // Volts
#define MAX_ACCEL_STRAIGHT 3 // m/s/s  
#define MAX_DECEL_STRAIGHT -3 // m/s/s
#define MAX_ACCEL_ROTATE 3 // m/s/s  
#define MAX_DECEL_ROTATE -3 // m/s/s
#define MAX_ACCEL_CORNER 3 // m/s/s  
#define MAX_DECEL_CORNER -3 // m/s/s


#define GEAR_RATIO 9.96 // gear ratio between motor and wheels
#define MAX_VEL_STRAIGHT .2 // m/s   limited by the maximum velocity at which motors can deliver max accel
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
#define MAX_VEL_ROTATE 0.1 // m/s
#define MAX_VEL_CORNER .5 // m/s

#endif
