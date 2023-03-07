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

    //Turret's motor, uses robot specific config for port
    Motor motor(TURRET_MOTOR, E_MOTOR_GEARSET_06, false, E_MOTOR_ENCODER_ROTATIONS);
    
    //Turret's limit switch, uses robot specific config for port
    ADIDigitalIn limit_switch(TURRET_LIMIT_SWITCH);

    //Target angle for turret to face
    double target_angle = 0.0; 
    //Turret speed limit in RPM
    double max_velocity = 0.0; 
    //Whether or not the vision system is working
    bool vision_working = true; 

    //Conversion factor from motor rotations to degrees
    const double ROT_TO_DEG = 37.5;
    //Left limit of turret in degrees
    const double LEFT_LIMIT = 80.0;
    //Right limit of turret in degrees
    const double RIGHT_LIMIT = -80.0; 
    //How close the turret needs to be to the target angle to be settled
    const double SETTLE_THRESHHOLD = 1.0; 

    /**
     * Inline function to convert from rotations to degrees
     */
    inline double rot_to_deg(double rot) { return rot * ROT_TO_DEG; }
    
    /**
     * Inline function to convert from degrees to rotations
     */
    inline double deg_to_rot(double deg) { return deg / ROT_TO_DEG; }

    /**
     * Enumerated class containing the state of the turret
     * 
     * DISABLED: Turret control system disabled
     * 
     * MANUAL: Manual control (vision control disabled)
     * 
     * VISION: Vision system control
     */
    enum class State {
        DISABLED,
        MANUAL,
        VISION
    };

    //Current state of the turret
    State state = State::MANUAL; //Set state to manual by default

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
        pros::lcd::print(0, "TURRET");
        if(state == State::DISABLED) {
            pros::lcd::print(1, " State: Disabled");
            return;
        } else if(state == State::MANUAL) {
            pros::lcd::print(1, " State: Manual");
        } else if(state == State::VISION) {
            pros::lcd::print(1, " State: Vision");
        }
        pros::lcd::print(2, " Vision Status: %s", vision_working ? "OPERATIONAL" : "SUSPENDED, ERROR DETECTED!");
        pros::lcd::print(3, " Current Angle: %f", get_angle());
        pros::lcd::print(4, " Target Angle: %f", target_angle);
        pros::lcd::print(5, " Angle Error: %f", get_angle_error());
        pros::lcd::print(6, " Settled: %s", settled() ? "True" : "False");
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

    void disable_turret() {
        state = State::DISABLED;
    }
}