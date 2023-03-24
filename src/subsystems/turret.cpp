#include "main.h"
#include "subsystems/subsystems.hpp"
#include "ARMS/config.h"
#include "turret.hpp"
#include "vision.hpp"

using namespace pros;

namespace turret {

    /**
    *
    * PUBLIC DATA (see header file for documentation)
    *
    */

    namespace{ //Anonymous namespace for private methods and data

        /*
        *
        * PRIVATE DATA
        *
        */

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

        //CONTROLLER CONSTANTS
        const double kP = TURRET_KP;
        const double kI = TURRET_KI;
        const double kD = TURRET_KD;
        const bool use_ff = TURRET_FF;
        const bool use_antiwindup = TURRET_AW;
        const double ff_voltage = TURRET_FF_V;
        //Local variables
        double integral = 0;
        double last_error = 0;
        double last_heading = 0;

        //Current state of the turret
        State state = State::MANUAL; //Set state to manual by default
        //Current target color

        #define TURRET_DEBUG true
        int printCounter = 0;

        /*
        *
        * PRIVATE METHODS
        *
        */

        /**
        * Inline function to convert from rotations to degrees
        */
        inline double rot_to_deg(double rot) { return rot * ROT_TO_DEG; }
        
        /**
        * Inline function to convert from degrees to rotations
        */
        inline double deg_to_rot(double deg) { return deg / ROT_TO_DEG; }

        /**
         * Checks if the vision system sees a valid target (based on the target color)
         * CONTAINS CHECK FOR VISION BROKEN
         * 
         * @return True if the last seen goal is a valid target, false otherwise
         */
        bool is_valid_target(){
            
            return false; //If vision is not working, return false
        }

        /**
         * PID and feedforward calculation for vision control
         * 
         * @param angle_error The angle error between the turret and the goal
         * @return The voltage to apply to the turret motor (mV)
         */
        double get_vision_voltage(double angle_error){
            //Calculate PID output:
            double pOut = kP * angle_error; //proportional term
            //Anti-windup for integral term:
            if(use_antiwindup){
                //Prevents i from increasing if it would just push the total output beyond max motor voltage
                integral += angle_error * fabs(pOut) < 12000; //(abs value thing is 1 or 0 depenidng on if already saturated)
            } else{ 
                //basic version if antiwindup is turned off for some reason
                integral += angle_error;
            }
            double iOut = kI * integral; //integral term
            double dOut = kD * (angle_error - last_error); //derivative term
            last_error = angle_error; //update last error
            double output = pOut + iOut + dOut; //output = sum of PID terms

            //Calculate feedforward output:
            if(use_ff){
                //Add feedforward term to output (feedforward voltage times change in heading since last calculation)
                output += ff_voltage * (arms::odom::getHeading() - last_heading);
                last_heading = arms::odom::getHeading(); //update last heading
            }

            //If the turret is settled, return 0mV, otherwise return the calculated output:
            return settled()? 0 : output; 
        }

        /**
         * Asynchronous task loop for turret control
         */
        void task_func() { 
            while(true) {
                //Get the angle error between the turret and the goal

                //Switch for desired control mode
                switch(state) {
                    case State::DISABLED: //Emergency stop basically
                        motor.move_voltage(0);
                        target_angle = 0;
                        break;
                    case State::MANUAL: //Manual control
                        motor.move_absolute(target_angle, 100);
                        break;
                    case State::VISION: //Vision control
                        // If the vision system is working, enable vision control
                        double error = vision::get_error();
                        target_angle = get_angle(false) - error;
                        double target = get_vision_voltage(error);
                        motor.move_voltage(target);
                        if(TURRET_DEBUG && printCounter++ % 5 == 0){
                            printf("\nTurret Error: %3.2f, Target: %5.2f", error, target);
                        }
                        break;
                }
                //Loop delay
                pros::delay(10);
            }
        }
    }

    /*
    *
    * PUBLIC METHODS (see header file for documentation)
    *
    */

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

    double get_angle(bool radians) {
        return rot_to_deg(motor.get_position()) * (radians? M_PI/180 : 1);
    }

    void goto_angle(double angle, double velocity, bool async) {
        // Clamp the angle so that we don't try to move to a position that will 
        // harm the ring gear or burn out the motor
        target_angle = std::clamp(angle, RIGHT_LIMIT, LEFT_LIMIT);
        max_velocity = velocity;

        if(!async){
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
        if(vision::get_targ_goal() == vision::Goal::BOTH){
            pros::lcd::print(2, " Target Color: Both");
        } else if(vision::get_targ_goal() == vision::Goal::RED){
            pros::lcd::print(2, " Target Color: Red");
        } else if(vision::get_targ_goal() == vision::Goal::BLUE){
            pros::lcd::print(2, " Target Color: Blue");
        }
        pros::lcd::print(3, " Vision Status: %s", vision::is_working() ? "OPERATIONAL" : "SUSPENDED, NO IRIS DATA!");
        pros::lcd::print(4, " Current Angle: %f", get_angle());
        pros::lcd::print(5, " Target Angle: %f", target_angle);
        pros::lcd::print(6, " Angle Error: %f", get_angle_error());
        pros::lcd::print(7, " Settled: %s", settled() ? "True" : "False");
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
} //End namespace turret