#ifndef CONF_H
#define CONF_H

#include <Arduino.h>

// Serial port baud rate
#define BAUD 9600

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

#define EMITTER_DIAG_LEFT_PIN 24
#define EMITTER_DIAG_RIGHT_PIN 25
#define EMITTER_FRONT_LEFT_PIN 12
#define EMITTER_FRONT_RIGHT_PIN 33

#define RANGE_DIAG_LEFT_PIN A11
#define RANGE_DIAG_RIGHT_PIN A12
#define RANGE_FRONT_LEFT_PIN A13
#define RANGE_FRONT_RIGHT_PIN A10

#define BUZZER_PIN 3

#define BATTERY_PIN A14

#define DISPLAY_DATA_PIN 15
#define DISPLAY_RS_PIN 27
#define DISPLAY_CLOCK_PIN 31
#define DISPLAY_ENABLE_PIN 14
#define DISPLAY_RESET_PIN 28

// RANGE SENSOR OPTIONS

// Translation formula constants. distance = a * (reading - b)^c + d
#define RANGE_DIAG_LEFT_TRANSLATION { 204279.127508, -140.177058331, -1.17485565668, 4.26428490432, -0.0004, 774., 2.29, 68., 774.051196698, 0 }
#define RANGE_DIAG_RIGHT_TRANSLATION { 1599.06663151, 27.8878612466, -0.225900340991, -274.178607786, -0.0007, 745., 2.29, 85., 746.880064643, 0 }
#define RANGE_FRONT_LEFT_TRANSLATION { 982.981088769, 3.01477552114, -0.290806601873, -105.008228929, -0.0006, 880, 2.4, 33, 880.068342392, 0 }
#define RANGE_FRONT_RIGHT_TRANSLATION { 3003.86136476, -27.8333578541, -0.546648961093, -23.5867782255, -0.0005, 818, 2.26, 50, 819.033713553, 6 }

// Range sensor wall thresholds
#define DIAG_LEFT_LOW_THRESHOLD 333
#define DIAG_LEFT_HIGH_THRESHOLD 333
#define DIAG_RIGHT_LOW_THRESHOLD 332
#define DIAG_RIGHT_HIGH_THRESHOLD 332
#define FRONT_LEFT_LOW_THRESHOLD 190
#define FRONT_LEFT_HIGH_THRESHOLD 190
#define FRONT_RIGHT_LOW_THRESHOLD 201
#define FRONT_RIGHT_HIGH_THRESHOLD 201

// Range sensor middle readings
#define RANGE_DIAG_LEFT_MIDDLE 201
#define RANGE_DIAG_RIGHT_MIDDLE 209

// Range sensor front value at which we stop using diagonal sensors
#define RANGE_DIAG_CUTOFF_FRONT_DISTANCE 74

// Gyro parameters
#define GYRO_LSB_PER_DEG_PER_S 16.295
#define GYRO_CALIBRATION_SAMPLES 2000
#define GYRO_CALIBRATION_ROUNDS 3
#define GYRO_OFFSET_SETTING -43
#define GYRO_SECONDARY_OFFSET 11.0050

// Range sensor delays in us
#define RANGE_SENSOR_ON_TIME 50 // length of LED pulse
#define RANGE_SENSOR_OFF_TIME 250 // time between pulses

// Accelerometer parameters
#define ACCEL_LSB_PER_G 2048

// Magnetometer parameters
#define MAG_CYCLES_PER_UPDATE 5
#define MAG_COMPLEMENTARY_FILTER 0.00

// Motor stop failsafe parameters
#define FAILSAFE_GYRO_THRESHOLD 100 // in deg/s
#define FAILSAFE_GYRO_ANGLE 20 // in deg
#define FAILSAFE_ACCEL_THRESHOLD 10// in mg

// Display control parameters
#define DISPLAY_SIZE 4
#define DISPLAY_BRIGHTNESS 10 // between 0 and 15 inclusive

// Menu configuration
#define BUTTON_OK_PIN BUTTON1_PIN
#define BUTTON_BACK_PIN BUTTON2_PIN
#define MENU_STEP_ANGLE 35 // distance between options in degrees
#define MENU_DEAD_ZONE 1
#define MENU_RESTORING_FORCE 5

// Hand swipe start
#define HAND_SWIPE_START_TIME 500 // in millis
#define HAND_SWIPE_HEADING_TOLERANCE 0.05 // in degrees
#define HAND_SWIPE_FORWARD_RANGE 50
#define HAND_SWIPE_DIAG_RANGE 85

// Logging configuration
#define LOG_SIZE 500

// EEPROM save location
#define EEPROM_MAZE_LOCATION 2
#define EEPROM_MAZE_FLAG_LOCATION 1
#define EEPROM_INITIAL_DIRECTION_LOCATION 2000

// Motion control paremeters
#define MM_PER_BLOCK 180
#define MM_FROM_BACK_TO_CENTER 30
#define MM_PER_STEP 0.653868
#define MOTION_COLLECT_MM_PER_READING 1

// PID tuning parameters
#define KP_POSITION 35
#define KI_POSITION 0
#define KD_POSITION 20000

#define KP_ROTATION 0.0005//0.003
#define KI_ROTATION 0.00000000//85
#define KD_ROTATION 05//9

#define KP_RANGE 4.5
#define KI_RANGE 0
#define KD_RANGE 5000

#define KP_GYRO 0.10
#define KI_GYRO 0.00000800
#define KD_GYRO 0.00

#define KP_GYRO_FWD 0.0015
#define KI_GYRO_FWD 0
#define KD_GYRO_FWD 0

#define KP_HOLD_RANGE 2
#define KI_HOLD_RANGE 0
#define KD_HOLD_RANGE 0

// Robot characteristics
#define ROBOT_MASS .1302 // kilograms
#define MOMENT_OF_INERTIA 0.00015 //kg -m^2
#define MM_BETWEEN_WHEELS 74.5
#define MM_BETWEEN_WHEELS_ROTATE 70
#define NUMBER_OF_MOTORS 4
#define STEPS_PER_MOTOR_REV 12// the number of encoder steps we get per wheel revolution
#define BATTERY_VOLTAGE 8.1 // Volts
#define BATTERY_VOLTAGE_WARNING 7.7 // Volts
#define MAX_COEFFICIENT_FRICTION 1
#define MAX_ACCEL_STRAIGHT 7 // m/s/s
#define MAX_DECEL_STRAIGHT -5 // m/s/s
#define MAX_ACCEL_ROTATE 2 // m/s/s
#define MAX_DECEL_ROTATE -2 // m/s/s
#define MAX_ACCEL_CORNER 3 // m/s/s  
#define MAX_DECEL_CORNER -3 // m/s/s


#define GEAR_RATIO 9.96 // gear ratio between motor and wheels
#define MAX_VEL_STRAIGHT 0.3 // m/s   limited by the maximum velocity at which motors can deliver max accel
#define PWM_SPEED_STEPS 1023 // maximum PWM value for the system

// Motor spec sheet parameters
#define RATED_VOLTAGE 8 // Voltage that ratings were measured with
#define RATED_INTERNAL_RESISTANCE 3.1 // ohms of resistance in motor coils
#define RATED_FREERUN_CURRENT 0.06// Amps
#define RATED_RPM_PER_VBEMF 500 // RPM/Volt
#define RATED_TORQUE_PER_AMP 0.000622353 // torque in N-m at output shaft, Amps passed through motor

// Convert motor parameters to robot parameters
#define WHEEL_RADIUS (MM_PER_STEP * STEPS_PER_MOTOR_REV * GEAR_RATIO / (2 * 3.14159265359))// in millimeters
#define FORCE_PER_AMP (RATED_TORQUE_PER_AMP * GEAR_RATIO * NUMBER_OF_MOTORS * 1000 / WHEEL_RADIUS) // in Newtons
#define VELOCITY_PER_VBEMF (RATED_RPM_PER_VBEMF * STEPS_PER_MOTOR_REV * GEAR_RATIO * MM_PER_STEP / (60000)) // 60000 is for mm to m and s to min
#define FRICTION_FORCE (RATED_FREERUN_CURRENT * FORCE_PER_AMP + 0.10) // Newtons (0.08 calculated Newtons from motor/gearbox)  amount of force opposing motion in robot including rolling resistance, sliding, gearing

//  TODO precompile, calculate max velocity based on turn radius and max accel, which will then limit max velocity through centripital force.  
//    if this max velocity is higher than max straight velocity then use max straight velocity
#define MAX_VEL_ROTATE .5 // m/s

#define KAOS_TURN_VEL 0.4
#define KAOS_FORWARD_VEL 0.4

#define MOTION_RESET_BACKUP_VEL 0.05
#define MOTION_RESET_BACKUP_DISTANCE 50
#define MOTION_RESET_HOLD_DISTANCE 22

// Zll forward speeds should be the same, and should be the maximum turn speed.  
// All max angular accelerations can be the same, but don't have to be
// highest reliable max angular accel is most accurate turn
//  max angular accel can be calculated with below equation
//  max angular accel = (max linear accel) * 90000 * ROBOT_MASS * MM_BETWEEN_WHEELS / (robot rot. inertia)
#define SWEPT_TURN_45_FORWARD_SPEED 0.87
#define SWEPT_TURN_45_ANGLE 45.0

#define SWEPT_TURN_90_FORWARD_SPEED 0.84
#define SWEPT_TURN_90_ANGLE 90.0

#define SWEPT_TURN_135_FORWARD_SPEED 0.8975
#define SWEPT_TURN_135_ANGLE 135.0

#define SWEPT_TURN_180_FORWARD_SPEED 0.935
#define SWEPT_TURN_180_ANGLE 180.0

#define SEARCH_VELOCITY 0.3

#endif
