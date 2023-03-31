#include "main.h"
#include "ARMS/config.h"
#include "subsystems.hpp"

using namespace pros;

namespace discLift {
    
    namespace{ //Anonymous namespace for private data and methods

        /**
        *
        * PRIVATE DATA
        * 
        */

        //The motor used to move the disc lift
        pros::Motor lift_motor(LIFT_MOTOR, pros::E_MOTOR_GEARSET_36, true, pros::E_MOTOR_ENCODER_DEGREES);
        //State variable: turns true once a disc reaches the indexer wheels;
        //remains true until that disc is fired
        bool lifted = false; 
        //State variable: turns true once the flywheel reaches speed;
        //remains true until the flywheel drops below speed AND the above state variable is true
        bool reachedSpeed = false;
        //The position of the lift when the lifted state variable switches to true
        float liftedPos;

        //NO PRIVATE METHODS
    }
    
    /**
    *
    * PUBLIC METHODS (see header file for documentation)
    * 
    */

    void discLiftUp(){
        lift_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        // Conditions for various states of the disc lift
        //Prevents lifted from changing back to false momentariily, once it's set it stays until 
        //the lift is lifted AND the flyweel detects a shot.
        if(!lifted){
            lifted = lift_motor.get_actual_velocity() < 2 && lift_motor.get_position() > 12;
            if(lifted){
                liftedPos = lift_motor.get_position();
            }
        } else if (lift_motor.get_actual_velocity() > 2){
            lifted = false;
        }

        if(!reachedSpeed){
            reachedSpeed = flywheel::at_speed();
        } else if (!flywheel::at_speed() && lifted){
            //Status changed + DL was up; shot detected
            lifted = false;
            reachedSpeed = false;
        }

        if(lifted){
            //DISC LIFT ALL THE WAY UP FOR CURRENT NUM OF DISCS
            lift_motor.move_absolute(liftedPos, 100);
        } else if(lift_motor.get_position() < LIFT_UP_POS-5){
            lift_motor.move_voltage(12000);
        } else{
            lift_motor.move_absolute(liftedPos, 100);
        }
    }
    
    void discLiftHold(){
        lift_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        if(lift_motor.get_position() < LIFT_UP_POS){
            lift_motor.move_voltage(7000);
            // lift_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
            // lift_motor.brake();
        } else{
            lift_motor.move_absolute(liftedPos,100);
            // lift_motor.brake();
        }
    }

    void discLiftDown(){
        lifted = false;
        reachedSpeed = false;
        lift_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        if(lift_motor.get_position() < LIFT_DOWN_POS - 2){
            // Acceptable tolerance, avoid burnout
            lift_motor.move_voltage(0);
        } else{
            lift_motor.move_absolute(LIFT_DOWN_POS -3, 100);
        }
    }

    void home() {
        intake::start(600);
        lift_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        lift_motor.move_voltage(12000);
        pros::delay(100);
        lift_motor.move_voltage(-5000);
        intake::stop();
        pros::delay(100);
        int timestamp = pros::millis();
        while (lift_motor.get_current_draw() < 1200) {
            pros::delay(10);
        }
        lift_motor.tare_position();
        double offset = LIFT_HOME_OFFSET;
        lift_motor.move_absolute(offset, 100);
        while(lift_motor.get_position() < offset){
            pros::delay(10);
        }
        lift_motor.move(0);
        pros::delay(200);
        lift_motor.tare_position();
    }

    void debug_screen(){
        pros::lcd::print(0, "Disc Lift Info: ");
        pros::lcd::print(1, " Lift Position: %3.2f", lift_motor.get_position());
        pros::lcd::print(2, " Lift Velocity: %3.2f", lift_motor.get_actual_velocity());
        pros::lcd::print(3, " Lift Current: %3d", lift_motor.get_current_draw());
        pros::lcd::print(4, " Lift Temperature: %3.2f", lift_motor.get_temperature());
        pros::lcd::print(5, " Lifted: %s", lifted?"true":"false");
        pros::lcd::print(6, " Reached Speed: %s", reachedSpeed?"true":"false");
    }
}