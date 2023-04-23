/**
This file includes all of the ARMS constants as well as all robot-specific constants for the GOLD robot.
*/
#pragma once

#define USING_BEN_PNEUMATICS true

#define ADI_EXPANDER 15
#define INTAKE_LEFT 11
#define INTAKE_RIGHT 19

#define ROLLER_MOTOR 6
#define ROLLER_OPTICAL 5

#define LIFT_MOTOR 21
#define LIFT_UP_POS 90
#define LIFT_DOWN_POS 7
#define LIFT_HOME_OFFSET 10


#define FLYWHEEL_THRESHOLD 100
#define FLYWHEEL_LEFT 5
#define FLYWHEEL_RIGHT 10
#define INDEXER_PORT 14


#define TURRET_MOTOR 7
#define JOSH_LAT_COMP false
#define TURRET_LIMIT_SWITCH 'h'
#define TURRET_KP 1000 //ONLY TUNE WITH A DISC IN THE ROBOT
#define TURRET_KI 0 //ONLY TUNE WITH A DISC IN THE ROBOT
#define TURRET_KD 200 //ONLY TUNE WITH A DISC IN THE ROBOT
#define TURRET_AW true //switch for anti integral windup
#define TURRET_FF true //switch for turn feedforward
#define TURRET_FF_V 4000.0 //feedforward voltage (mV)
#define TURRET_MIN_V 565 //minimum voltage to move(mV)
#define TURRET_MAX_V 10000 //maximum allowed velocity out of 600 (rpm)


#define PNEUMATICS_PORT 'g'
#define LEFT_ENDGAME 2
#define RIGHT_ENDGAME 4
#define BLOCKER 1
#define DEFLECTOR 3

#define IRIS_PORT 15


// Debug
#define ODOM_DEBUG 0

#define LEFT_MOTORS 3, -13, 12
#define RIGHT_MOTORS -16, 17, -18
#define GEARSET pros::E_MOTOR_GEAR_600 // RPM of chassis motors

// Ticks per inch
#define TPI 320.00       // Encoder ticks per inch of forward robot movement
#define MIDDLE_TPI 320.0 // Ticks per inch for the middle wheel

// Tracking wheel distances
#define TRACK_WIDTH                                                            \
  5.75 // The distance between left and right wheels (or tracker wheels)
#define MIDDLE_DISTANCE                                                        \
  6.25 // Distance from middle wheel to the robot turning center

// Sensors
#define IMU_PORT 0                           // Port 0 for disabled
#define ENCODER_PORTS 3, 5, -1                // Port 0 for disabled,
#define EXPANDER_PORT 0                      // Port 0 for disabled
#define ENCODER_TYPE arms::odom::ENCODER_ADI // The type of encoders

// Movement tuning
#define SLEW_STEP 4             // Smaller number = more slew
#define LINEAR_EXIT_ERROR 5     // default exit distance for linear movements
#define ANGULAR_EXIT_ERROR 5    // default exit distance for angular movements
#define SETTLE_THRESH_LINEAR .5 // amount of linear movement for settling
#define SETTLE_THRESH_ANGULAR 1 // amount of angular movement for settling
#define SETTLE_TIME 250         // amount of time to count as settled
#define LINEAR_KP 10
#define LINEAR_KI 0
#define LINEAR_KD 30
#define TRACKING_KP 70 // point tracking turning strength
#define ANGULAR_KP 5
#define ANGULAR_KI 0.1
#define ANGULAR_KD 37
#define MIN_ERROR                                                              \
  5 // Minimum distance to target before angular componenet is disabled
#define LEAD_PCT .6 // Go-to-pose lead distance ratio (0-1)

// Auton selector configuration constants
#define AUTONS "Front", "Back", "Do Nothing" // Names of autonomi, up to 10
#define HUE 0     // Color of theme from 0-359(H part of HSV)
#define DEFAULT 1 // Default auton selected
