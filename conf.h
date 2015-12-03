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

// Motion control paremeters
#define MM_PER_BLOCK 180
#define MM_PER_STEP 0.7928

// Robot characteristics
#define ROBOT_MASS .15 // kilograms
#define WHEEL_RADIUS .015 // meters
#define MM_BETWEEN_WHEELS 75 // milimeters
#define NUMBER_OF_MOTORS 2
#define FRICTION_COEFFICIENT 1 // Coefficient of friciton between wheels and ground
#define MAX_ACCEL 4 // m/s/s  
#define MAX_DECEL -4 // m/s/s
#define MAX_VELOCITY_STRAIGHT 1 // m/s   limited by the maximum velocity at which motors can deliver max accel

//  TODO precompile, calculate max velocity based on turn radius and max accel, which will then limit max velocity through centripital force.  
//    if this max velocity is higher than max straight velocity then use max straight velocity
#define MAX_VELOCITY_ROTATE .05 // m/s

// Motor spec sheet parameters
//  TODO precompile, calculate max force per wheel from stall torque with wheel radius, then rated free run speed based on 
#define RATED_STALL_TORQUE 0.02824620733332 // Newton-meters
#define RATED_FREERUN_SPEED 314.159265 // radians/second
#define RATED_VOLTAGE 6 // Volts

#endif
