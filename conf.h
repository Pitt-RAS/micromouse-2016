#ifndef CONF_H
#define CONF_H

#include <Arduino.h>

// Define pin assignments for teensy3.2 on Rev2 PCB
#define ENCODER_LF1_PIN 4
#define ENCODER_LF2_PIN 2
#define MOTOR_LF_DIRECTION_PIN 16
#define MOTOR_LF_PWM_PIN 20
#define MOTOR_LF_FORWARD_STATE HIGH

#define ENCODER_RF1_PIN 8
#define ENCODER_RF2_PIN 7
#define MOTOR_RF_DIRECTION_PIN 18
#define MOTOR_RF_PWM_PIN 22
#define MOTOR_RF_FORWARD_STATE LOW

#define ENCODER_LB1_PIN 5
#define ENCODER_LB2_PIN 6
#define MOTOR_LB_DIRECTION_PIN 17
#define MOTOR_LB_PWM_PIN 21
#define MOTOR_LB_FORWARD_STATE LOW

#define ENCODER_RB1_PIN 9
#define ENCODER_RB2_PIN 10
#define MOTOR_RB_DIRECTION_PIN 19
#define MOTOR_RB_PWM_PIN 23
#define MOTOR_RB_FORWARD_STATE HIGH

#define BUTTON1_PIN 26
#define BUTTON2_PIN 32

#define SCL_PIN 29
#define SDA_PIN 30
#define IMU_INTERRUPT_PIN 11

#define EMITTER1_PIN 24
#define EMITTER2_PIN 25
#define EMITTER3_PIN 12
#define EMITTER4_PIN 33

#define RANGE1_PIN A11
#define RANGE2_PIN A12
#define RANGE3_PIN A13
#define RANGE4_PIN A10

#define BUZZER_PIN 3

#define BATTERY_PIN A14

#define DISPLAY_DATA_PIN 15
#define DISPLAY_RS_PIN 27
#define DISPLAY_CLOCK_PIN 31
#define DISPLAY_ENABLE_PIN 14
#define DISPLAY_RESET_PIN 28

// RANGE SENSOR OPTIONS

// Translation formula constants. distance = a * (reading - b)^c + d
#define RANGE1_TRANSLATION { 845.177, -14.9819, -0.337217, -46.471, 0 }
#define RANGE2_TRANSLATION { 0.0, 0.0, 0.0, 0.0, 0 }
#define RANGE3_TRANSLATION { 4905.88, 27.0265, -0.672239, -87.2552, 0 }
#define RANGE5_TRANSLATION { 1144.64, -13.4041, -0.389189, -103.488, 0 }

// Range sensor directions
#define RANGE_DIAG_LEFT_PIN RANGE3_PIN
#define RANGE_DIAG_RIGHT_PIN RANGE5_PIN
#define RANGE_LEFT_PIN RANGE3_PIN
#define RANGE_RIGHT_PIN RANGE4_PIN
#define RANGE_FRONT_PIN RANGE1_PIN

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

#define LEFT_LOW_THRESHOLD 150
#define LEFT_HIGH_THRESHOLD 150
#define RIGHT_LOW_THRESHOLD 150
#define RIGHT_HIGH_THRESHOLD 150
#define FRONT_LOW_THRESHOLD 90
#define FRONT_HIGH_THRESHOLD 90

// Range sensor middle readings
#define RANGE_DIAG_LEFT_MIDDLE 100
#define RANGE_DIAG_RIGHT_MIDDLE 100
#define RANGE_LEFT_MIDDLE 100
#define RANGE_RIGHT_MIDDLE 100
#define RANGE_FRONT_MIDDLE 100

// Display control parameters
#define DISPLAY_SIZE 4
#define DISPLAY_BRIGHTNESS 10 // between 0 and 15 inclusive

// Menu configuration
#define BUTTON_OK_PIN BUTTON1_PIN
#define BUTTON_BACK_PIN BUTTON2_PIN
#define MENU_STEP_ANGLE 35 // distance between options in degrees
#define MENU_DEAD_ZONE 1
#define MENU_RESTORING_FORCE 2

// Motion control paremeters
#define MM_PER_BLOCK 180
#define MM_PER_STEP 0.6444

// PID tuning parameters
#define KP_POSITION 5.69
#define KI_POSITION 0
#define KD_POSITION 0

#define KP_ROTATION 1
#define KI_ROTATION 0
#define KD_ROTATION 0

#define KP_HOLD_RANGE 0.5
#define KI_HOLD_RANGE 0
#define KD_HOLD_RANGE 0

// Robot characteristics
#define ROBOT_MASS .1302 // kilograms
#define MOMENT_OF_INERTIA 0.00015 //kg -m^2
#define MM_BETWEEN_WHEELS 74.5 // millimeters
#define NUMBER_OF_MOTORS 4
#define STEPS_PER_MOTOR_REV 12// the number of encoder steps we get per wheel revolution
#define BATTERY_VOLTAGE 8.1 // Volts
#define MAX_COEFFICIENT_FRICTION 1
#define MAX_ACCEL_STRAIGHT 10 // m/s/s
#define MAX_DECEL_STRAIGHT -7.6 // m/s/s
#define MAX_ACCEL_ROTATE 3 // m/s/s  
#define MAX_DECEL_ROTATE -3 // m/s/s
#define MAX_ACCEL_CORNER 3 // m/s/s  
#define MAX_DECEL_CORNER -3 // m/s/s


#define GEAR_RATIO 9.96 // gear ratio between motor and wheels
#define MAX_VEL_STRAIGHT 3.5 // m/s   limited by the maximum velocity at which motors can deliver max accel
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
#define FRICTION_FORCE (RATED_FREERUN_CURRENT * FORCE_PER_AMP + 0.10) // Newtons (0.08 calculated Newtons from motor/gearbox)  amount of force opposing motion in robot including rolling resistance, sliding, gearing

//  TODO precompile, calculate max velocity based on turn radius and max accel, which will then limit max velocity through centripital force.  
//    if this max velocity is higher than max straight velocity then use max straight velocity
#define MAX_VEL_ROTATE 0.1 // m/s
#define MAX_VEL_CORNER 0.5 // m/s


// Zll forward speeds should be the same, and should be the maximum turn speed.  
// All max angular accelerations can be the same, but don't have to be
// highest reliable max angular accel is most accurate turn
//  max angular accel can be calculated with below equation
//  max angular accel = (max linear accel) * 90000 * ROBOT_MASS * MM_BETWEEN_WHEELS / (robot rot. inertia)
#define SWEPT_TURN_45_FORWARD_SPEED 0.8
#define SWEPT_TURN_45_ANGLE 45.0

#define SWEPT_TURN_90_FORWARD_SPEED 0.84
#define SWEPT_TURN_90_ANGLE 90.0

#define SWEPT_TURN_135_FORWARD_SPEED 0.8975
#define SWEPT_TURN_135_ANGLE 135.0

#define SWEPT_TURN_180_FORWARD_SPEED 0.935
#define SWEPT_TURN_180_ANGLE 180.0

#endif
