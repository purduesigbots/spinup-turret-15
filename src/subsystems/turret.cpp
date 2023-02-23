#include "subsystems/turret.hpp"

#include "subsystems/subsystems.hpp"
#include "api.h"

#include <cmath>
#include <algorithm>

using namespace pros;

namespace turret {

// Devices needed for implementing the subsystem:
Motor motor(7, E_MOTOR_GEARSET_06, true, E_MOTOR_ENCODER_ROTATIONS);
ADIDigitalIn limit_switch('e');

double target_angle = 0.0;

// This constant is used to convert the motor's possition, which is in
// rotations, to the turret's angle in degrees.
const double ROT_TO_DEG = 37.5;

const double LEFT_LIMIT = 80.0;         // Maximum angle to the left
const double RIGHT_LIMIT = -80.0;       // Maximum angle to the right 

const double SETTLE_THRESHHOLD = 0.5;   // How many degrees to the left or right
                                        // do we consider within the settled
                                        // range

inline double rot_to_deg(double rot) { return rot * ROT_TO_DEG; }
inline double deg_to_rot(double deg) { return deg / ROT_TO_DEG; }

void initialize() {
    calibrate();
}

void calibrate() {
    // Set the motor to move to the left
    motor.move(50);

    // Wait until the limit switch is hit. This ensures the turret stops at a 
    // consistent location
    while(!limit_switch.get_value()) {
        pros::delay(20);
    }

    // Stop the motor so it doesn't break the ring gear
    motor.move(0);

    // Now tell the motor to move back to face forward.
    motor.move_relative(-2.24, 250);
    pros::delay(1000);
    motor.move(0);

    // Tare the position so that forward is 0.0
    motor.tare_position();
    pros::delay(100);
}

double get_angle() {
    return rot_to_deg(motor.get_position());
}

void goto_angle(double angle, double velocity, bool async) {
    // Clamp the angle so that we don't try to move to a position that will 
    // harm the ring gear or burn out the motor
    target_angle = std::clamp(deg_to_rot(angle), RIGHT_LIMIT, LEFT_LIMIT);

    // Let PROS handle the positioning of the motor 
    motor.move_absolute(target_angle, velocity);

    if(!async) {
        wait_until_settled();
    }
}

void goto_rel_move(double angle, double velocity, bool async) {
    goto_angle(get_angle() + angle, velocity, async);
}

double get_angle_error() {
    return get_angle() - target_angle;
}

bool settled() {
    return fabs(get_angle_error()) < SETTLE_THRESHHOLD;
}

/**
 * Blocks execution until the turret reaches the point it is supposed to. 
 */
void wait_until_settled() {
    // While the target_angle is outside the range we want, we sleep.
    // Once it is within SETTLE_THRESHHOLD degrees of the target angle, we quit
    // the loop.
    while(!settled()) {
        printf("Turret not settled:\n");
        printf("    Delta: %f\n", get_angle_error);
        pros::delay(10);
    }
}

} // namespace turret