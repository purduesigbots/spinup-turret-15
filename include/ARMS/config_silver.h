/**
This file includes all of the ARMS constants as well as all robot-specific
constants for the SILVER robot.
*/
#pragma once

#define INTAKE_PISTON                                                          \
	{                                                                            \
		{ 3, 'd' }                                                                 \
	}
#define INTAKE_LEFT 11
#define INTAKE_RIGHT 19
#define INTAKE_LINE 'h'

#define ROLLER_MOTOR 10
#define ROLLER_OPTICAL 5

#define LIFT_MOTOR 16
#define LIFT_UP_POS 110
#define LIFT_DOWN_POS 7

#define FLYWHEEL_THRESHOLD 200
#define FLYWHEEL_KV 58
#define FLYWHEEL_KP 20
#define FLYWHEEL_KI 0.001
#define FLYWHEEL_KD 0
#define FLYWHEEL_LEFT 8
#define FLYWHEEL_RIGHT 9
#define INDEXER_PORT 15

#define TURRET_MOTOR -7
#define TURRET_LIMIT_SWITCH                                                    \
	{                                                                            \
		{ 3, 'h' }                                                                 \
	}
#define TURRET_KP 300
#define TURRET_KI 0.01
#define TURRET_KD 0
#define TURRET_AW true;  // switch for anti integral windup
#define TURRET_FF true;  // switch for turn feedforward
#define TURRET_FF_V 0.0; // feedforward voltage (mV)

#define DEFLECTOR_PISTON                                                       \
	{                                                                            \
		{ 3, 'a' }                                                                 \
	}
#define ENDGAME_PISTON                                                         \
	{                                                                            \
		{ 3, 'd' }                                                                 \
	}

#define IRIS_PORT 21

// Debug
#define ODOM_DEBUG 0

// Negative numbers mean reversed motor

#define LEFT_MOTORS 14, -13, 12
#define RIGHT_MOTORS -17, 18, -20
#define GEARSET pros::E_MOTOR_GEAR_600 // RPM of chassis motors

// Ticks per inch
#define TPI 326.3157        // Encoder ticks per inch of forward robot movement
#define MIDDLE_TPI 326.3157 // Ticks per inch for the middle wheel

// Tracking wheel distances
#define TRACK_WIDTH                                                            \
	5.925 // The distance between left and right wheels (or tracker wheels)
#define MIDDLE_DISTANCE                                                        \
	6.25 // Distance from middle wheel to the robot turning center

// Sensors
#define IMU_PORT 0                           // Port 0 for disabled
#define ENCODER_PORTS -1, -5, 3              // Port 0 for disabled,
#define EXPANDER_PORT 0                      // Port 0 for disabled
#define ENCODER_TYPE arms::odom::ENCODER_ADI // The type of encoders

// Movement tuning
#define SLEW_STEP 4             // Smaller number = more slew
#define LINEAR_EXIT_ERROR 1     // default exit distance for linear movements
#define ANGULAR_EXIT_ERROR 1    // default exit distance for angular movements
#define SETTLE_THRESH_LINEAR .5 // amount of linear movement for settling
#define SETTLE_THRESH_ANGULAR 1 // amount of angular movement for settling
#define SETTLE_TIME 500         // amount of time to count as settled
#define LINEAR_KP 5
#define LINEAR_KI 0.01
#define LINEAR_KD 1.7
#define TRACKING_KP 80 // point tracking turning strength
#define ANGULAR_KP 5
#define ANGULAR_KI 0
#define ANGULAR_KD 35
#define MIN_ERROR                                                              \
	5 // Minimum distance to target before angular componenet is disabled
#define LEAD_PCT .6 // Go-to-pose lead distance ratio (0-1)

// Auton selector configuration constants
#define AUTONS "Front", "Back", "Do Nothing" // Names of autonomi, up to 10
#define HUE 0     // Color of theme from 0-359(H part of HSV)
#define DEFAULT 1 // Default auton selected
