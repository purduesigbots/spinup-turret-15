#include "main.h"
#include "subsystems/subsystems.hpp"
#include "ARMS/config.h"

using namespace pros;

namespace intake {

    namespace{ //Anonymus namespace for private data and methods

        /**
        *
        * PRIVATE DATA
        *
        */

        //Motors for the intake
        Motor left_motor(INTAKE_LEFT, E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_ROTATIONS);
        Motor right_motor(INTAKE_RIGHT, E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_ROTATIONS);

        //State variable; true if the intake is running, false otherwise
        bool running = false;
        //Local variable for intake target speed
        double speed = 0;

    } //End anonymous namespace

    /**
    *
    * PUBLIC METHODS (see header file for documentation)
    *
    */

    void start(double speed) {
        left_motor.move_voltage(120 * speed);
        right_motor.move_voltage(120 * speed);
        running = speed == 0? false : true;
        intake::speed = speed;
    }

    void stop() {
        left_motor.move_voltage(0);
        right_motor.move_voltage(0);
        running = false;
        intake::speed = 0.0;
    }

    void toggle(double speed) {
        if(!running) {
            start(speed);
        }
        else {
            stop();
        }
    }     

    bool intaking() {
        return speed > 0.0;
    }

    bool outtaking() {
        return speed < 0.0;
    }
}