#include "subsystems/turret.hpp"

#include "pros/adi.hpp"
#include "../../include/subsystems.h"
#include "main.h"
#include "vision.h"
#include "api.h"

#include <algorithm>

#if BOT == GOLD
  #include "ARMS/config_gold.h"
#elif BOT == SILVER
  #include "ARMS/config_silver.h"
#endif

using namespace pros;

namespace turret {

#define DISABLED 0
#define MOVE_TO_ANGLE 1
#define MOVE_WITH_VISION 2

// Devices needed for implementing the subsystem:

Motor motor(TURRET_MOTOR, E_MOTOR_GEARSET_06, false, E_MOTOR_ENCODER_ROTATIONS);
ADIDigitalIn limit_switch(TURRET_LIMIT_SWITCH);


double target_angle = 0.0;
double max_velocity = 0.0;
int state = DISABLED;
bool vision_working = true;

// This constant is used to convert the motor's possition, which is in
// rotations, to the turret's angle in degrees.
const double ROT_TO_DEG = 37.5;

const double LEFT_LIMIT = 80.0;         // Maximum angle to the left
const double RIGHT_LIMIT = -80.0;       // Maximum angle to the right 

const double SETTLE_THRESHHOLD = 1.0;   // How many degrees to the left or right
                                        // do we consider within the settled
                                        // range

void initialize() {
    motor.set_brake_mode(E_MOTOR_BRAKE_HOLD);

    // calibrate();
}

void calibrate() {
    // Set the motor to move to the left
    printf("Moving turret to the left\n");
    motor.move(100);

    // Wait until the limit switch is hit. This ensures the turret stops at a 
    // consistent location
    printf("Waiting for limit switch\n");
    while(!limit_switch.get_value()) {
        pros::delay(20);
    }

    // Stop the motor so it doesn't break the ring gear
    printf("Stopping motor\n");
    motor.move(0);
    pros::delay(100);
    // Now tell the motor to move back to face forward.
    printf("Moving to face forward\n");
    motor.move_relative(-2.24, 250);
    pros::delay(1000);
    motor.move(0);

    // Tare the position so that forward is 0.0
    printf("Taring position\n");
    motor.tare_position();
    pros::delay(100);
    printf("done\n");
}

double last_error = 0.0;

void update() {
    double angle_error = vision::get_goal_gamma();
    vision_working = angle_error != 45.00 && angle_error != last_error;
    last_error = angle_error;
    switch(state) {
        case DISABLED:
            motor.move(0);
            break;
        case MOVE_TO_ANGLE:
            motor.move_absolute(target_angle, max_velocity);
            break;
        case MOVE_WITH_VISION:
            /*
            Angle error is set to 45.00 when the goal is not detected
            If angle error has not changed, assume vision has disconnected
            */
            if (vision_working) {
                motor.move_voltage(angle_error * 400);
            } else {
                motor.move_voltage(0);
            }
            break;
    }
}

double get_angle() {
    return motor.get_position() * ROT_TO_DEG;
}

void goto_angle(double angle, double velocity, bool async) {
    // Clamp the angle so that we don't try to move to a position that will 
    // harm the ring gear or burn out the motor
    target_angle = std::clamp(angle / ROT_TO_DEG, RIGHT_LIMIT, LEFT_LIMIT);
    max_velocity = velocity;
    state = MOVE_TO_ANGLE;

    if(!async) {
        wait_until_settled();
    }
}

void toggle_vision_aim() {
    if (state == MOVE_WITH_VISION) {
        state = MOVE_TO_ANGLE;
    } else {
        state = MOVE_WITH_VISION;
    }
}

void enable_vision_aim() {
    state = MOVE_WITH_VISION;
}

void disable_vision_aim() {
    state = MOVE_TO_ANGLE;
}


bool settled() {
    return std::abs(get_angle() - target_angle) < SETTLE_THRESHHOLD;
}

/**
 * Blocks execution until the turret reaches the point it is supposed to. 
 */
void wait_until_settled() {
    // While the target_angle is outside the range we want, we sleep.
    // Once it is within SETTLE_THRESHHOLD degrees of the target angle, we quit
    // the loop.
    while(!settled()) {
        pros::delay(10);
    }
}

} // namespace turret