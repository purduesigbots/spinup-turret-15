#include "disclift.hpp"

#include "ARMS/config.h"

#include "api.h"
#include "subsystems.hpp"

using namespace pros;

namespace disclift {
    
    //LOCAL DEFS:
    pros::Motor lift_motor(LIFT_MOTOR, pros::E_MOTOR_GEARSET_36, true, pros::E_MOTOR_ENCODER_DEGREES);
    bool lifted = false; //if true, keep true until a disc is fired
    bool reachedSpeed = false;
    int targState = 0; // 0 = down, 1 = up, 2 = hold
    float liftedPos;

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
    
    void calculatePos(){
        // newPos = lift_motor.get_position();
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
        lift_motor.move_absolute(20, 100);
        while(lift_motor.get_position() < 20){
            pros::delay(10);
        }
        lift_motor.move(0);
        pros::delay(200);
        lift_motor.tare_position();
    }
}