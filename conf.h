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

// PID tuning parameters
#define KP .01
#define KD 0
#define KI 0

// Robot characteristics
#define ROBOT_MASS .15 // kilograms
#define MM_BETWEEN_WHEELS 75 // millimeters
#define FRICTION_FORCE 0.075// Newtons.  amount of force opposing motion in robot including rolling resistance, sliding, gearing
#define NUMBER_OF_MOTORS 2
#define STEPS_PER_WHEEL_REV 12// the number of encoder steps we get per wheel revolution
#define BATTERY_VOLTAGE 8.3 // Volts
#define MAX_ACCEL 4 // m/s/s  
#define MAX_DECEL -4 // m/s/s
#define GEAR_RATIO 9.96 // gear ratio between motor and wheels
#define MAX_VELOCITY_STRAIGHT 1 // m/s   limited by the maximum velocity at which motors can deliver max accel
#define PWM_SPEED_STEPS 255 // maximum PWM value for the system

// Motor spec sheet parameters
//  TODO precompile, calculate max force per wheel from stall torque with wheel radius, then rated free run speed based on 
#define RATED_STALL_TORQUE 0.00282462073 // Newton-meters before motor enters gearbox
#define RATED_FREERUN_RPM 30000 // RPM at motor shaft
#define RATED_VOLTAGE 6 // Volts

// calculate parameters based on above values
#define WHEEL_RADIUS MM_PER_STEP * STEPS_PER_WHEEL_REV / (2 * 3.14159265359)// in millimeters
#define RATED_FREERUN_VELOCITY RATED_FREERUN_RPM * MM_PER_STEP * STEPS_PER_WHEEL_REV / (60 * GEAR_RATIO * 1000) // 1000 is for mm in m, 60 is for s in min
#define RATED_STALL_FORCE RATED_STALL_TORQUE * GEAR_RATIO * NUMBER_OF_MOTORS * 1000 / WHEEL_RADIUS // in Newtons


//  TODO precompile, calculate max velocity based on turn radius and max accel, which will then limit max velocity through centripital force.  
//    if this max velocity is higher than max straight velocity then use max straight velocity
#define MAX_VELOCITY_ROTATE .05 // m/s





#endif
