#include "subsystems/turret.hpp"

#include "pros/adi.hpp"
#include "main.h"
#include "vision.h"
#include "api.h"

#include "ARMS/config.h"

#include <cmath>
#include <algorithm>

using namespace pros;

namespace turret {

// Devices needed for implementing the subsystem:

Motor motor(TURRET_MOTOR, E_MOTOR_GEARSET_06, false, E_MOTOR_ENCODER_ROTATIONS);
ADIDigitalIn limit_switch(TURRET_LIMIT_SWITCH);


double target_angle = 0.0;
double max_velocity = 0.0;
bool vision_working = true;

// This constant is used to convert the motor's possition, which is in
// rotations, to the turret's angle in degrees.
const double ROT_TO_DEG = 37.5;

const double LEFT_LIMIT = 80.0;         // Maximum angle to the left
const double RIGHT_LIMIT = -80.0;       // Maximum angle to the right 

const double SETTLE_THRESHHOLD = 1.0;   // How many degrees to the left or right
                                        // do we consider within the settled
                                        // range

inline double rot_to_deg(double rot) { return rot * ROT_TO_DEG; }
inline double deg_to_rot(double deg) { return deg / ROT_TO_DEG; }

enum class State {
    DISABLED,
    MANUAL,
    VISION
};
State state = State::MANUAL;

void task_func() {
    static double last_error = 0;
    static int settler = 0;

    while(true) {
        double angle_error = vision::get_goal_gamma();
        if (angle_error == last_error) {
            settler += 1;
        } else {
            settler = 0;
        }
        vision_working = angle_error != 45.00 && ! vision::vision_not_working();
        last_error = angle_error;
        switch(state) {
            case State::DISABLED:
                motor.move(0);
                break;
            case State::MANUAL:
                motor.move_absolute(deg_to_rot(target_angle), max_velocity);
                break;
            case State::VISION:
                /*
                Angle error is set to 45.00 when the goal is not detected
                If angle error has not changed, assume vision has disconnected
                */
                if (vision_working) {
                    motor.move_voltage(angle_error * 300);
                } else {
                    motor.move_voltage(0);
                }
                break;
        }

        pros::delay(10);
    }

}

void initialize() {
    calibrate();

    pros::Task task(task_func, "Turret Task");
}

void calibrate() {
    // Set the motor to move to the left
    printf("Moving turret to the left\n");
    motor.move(80);

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

double get_angle() {
    return rot_to_deg(motor.get_position());
}

void goto_angle(double angle, double velocity, bool async) {
    // Clamp the angle so that we don't try to move to a position that will 
    // harm the ring gear or burn out the motor
    target_angle = std::clamp(angle, RIGHT_LIMIT, LEFT_LIMIT);
    max_velocity = velocity;

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
    return fabs(get_angle_error()) <= SETTLE_THRESHHOLD;
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

void debug_screen() {
    if(state == State::DISABLED) {
        pros::lcd::print(0, "State: Disabled");
        return;
    } else if(state == State::MANUAL) {
        pros::lcd::print(0, "State: Manual");
    } else if(state == State::VISION) {
        pros::lcd::print(0, "State: Vision");
    }
    pros::lcd::print(1, " State:");
    pros::lcd::print(2, " Cur Angle: %f", get_angle());
    pros::lcd::print(3, " Tgt Angle: %f", target_angle);
    pros::lcd::print(4, " Angle Err: %f", get_angle_error());
    pros::lcd::print(5, " Settled: %s", settled() ? "true" : "false");
    pros::lcd::print(6, " Vision: %s", vision_working ? "true" : "false");
}

void toggle_vision_aim() {
    state = (state == State::VISION ? State::MANUAL : State::VISION);
}

void enable_vision_aim() {
    state = State::VISION;
}

void disable_vision_aim() {
    state = State::MANUAL;
}

} // namespace turret