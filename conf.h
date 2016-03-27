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
#define RANGE1_TRANSLATION { 994.826429322, 83.3221810758, -0.388831125077, -46.3836640345, -1.89043388124e-307, -4925.1855819, 81.8210698189, 35.6570927035, 760.662460695, 0 }
#define RANGE2_TRANSLATION { 994.826429322, 83.3221810758, -0.388831125077, -46.3836640345, -1.89043388124e-307, -4925.1855819, 81.8210698189, 35.6570927035, 760.662460695, 0 }
#define RANGE3_TRANSLATION { 931.451379287, 27.0728820886, -0.296932163326, -84.89798742, -2.20803389227e-307, -3082.76278937, 85.4472982988, 44.9908354769, 832.325325608, 0 }
#define RANGE4_TRANSLATION { 1184.70304319, 0.422585327611, -0.363408897551, -57.4731580317, -2.37592562318e-307, -2365.97981638, 87.4792518754, 46.2057278589, 851.029192427, 0 }

// Range sensor directions
#define RANGE_DIAG_LEFT_PIN RANGE1_PIN
#define RANGE_DIAG_RIGHT_PIN RANGE2_PIN
#define RANGE_FRONT_LEFT_PIN RANGE3_PIN
#define RANGE_FRONT_RIGHT_PIN RANGE4_PIN

// Range sensor distance offsets
#define RANGE_DIAG_LEFT_OFFSET -35
#define RANGE_DIAG_RIGHT_OFFSET -35
#define RANGE_LEFT_OFFSET 15
#define RANGE_RIGHT_OFFSET 15
#define RANGE_FRONT_OFFSET 30

// Range sensor wall thresholds
#define DIAG_LEFT_LOW_THRESHOLD 100
#define DIAG_LEFT_HIGH_THRESHOLD 100
#define DIAG_RIGHT_LOW_THRESHOLD 100
#define DIAG_RIGHT_HIGH_THRESHOLD 100
#define FRONT_LEFT_LOW_THRESHOLD 90
#define FRONT_LEFT_HIGH_THRESHOLD 90
#define FRONT_RIGHT_LOW_THRESHOLD 90
#define FRONT_RIGHT_HIGH_THRESHOLD 90

// Range sensor middle readings
#define RANGE_DIAG_LEFT_MIDDLE 68
#define RANGE_DIAG_RIGHT_MIDDLE 68
#define RANGE_LEFT_MIDDLE 100
#define RANGE_RIGHT_MIDDLE 100
#define RANGE_FRONT_MIDDLE 100

// Gyro parameters
#define GYRO_LSB_PER_DEG_PER_S 16.295
#define GYRO_CALIBRATION_SAMPLES 2000
#define GYRO_CALIBRATION_ROUNDS 3
#define GYRO_OFFSET_SETTING -43
#define GYRO_SECONDARY_OFFSET 10.5065

// Range sensor delays in us
#define RANGE_SENSOR_ON_TIME 50 // length of LED pulse
#define RANGE_SENSOR_OFF_TIME 250 // time between pulses

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
#define MOTION_COLLECT_MM_PER_READING 1

// PID tuning parameters
#define KP_POSITION 5.69
#define KI_POSITION 0
#define KD_POSITION 0

#define KP_ROTATION 0.001//0.003
#define KI_ROTATION 0.00000000//85
#define KD_ROTATION 5//9

#define KP_HOLD_RANGE 0.5
#define KI_HOLD_RANGE 0
#define KD_HOLD_RANGE 0

// Robot characteristics
#define ROBOT_MASS .1302 // kilograms
#define MOMENT_OF_INERTIA 0.00015 //kg -m^2
#define MM_BETWEEN_WHEELS 77.0//74.5 // millimeters
#define NUMBER_OF_MOTORS 4
#define STEPS_PER_MOTOR_REV 12// the number of encoder steps we get per wheel revolution
#define BATTERY_VOLTAGE 8.1 // Volts
#define BATTERY_VOLTAGE_WARNING 7.7 // Volts
#define MAX_COEFFICIENT_FRICTION 1
#define MAX_ACCEL_STRAIGHT 7 // m/s/s
#define MAX_DECEL_STRAIGHT -4 // m/s/s
#define MAX_ACCEL_ROTATE 3 // m/s/s  
#define MAX_DECEL_ROTATE -3 // m/s/s
#define MAX_ACCEL_CORNER 3 // m/s/s  
#define MAX_DECEL_CORNER -3 // m/s/s


#define GEAR_RATIO 9.96 // gear ratio between motor and wheels
#define MAX_VEL_STRAIGHT 0.1 // m/s   limited by the maximum velocity at which motors can deliver max accel
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
#define MAX_VEL_ROTATE .3 // m/s


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
