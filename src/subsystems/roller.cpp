#include "subsystems/roller.hpp"
#include "main.h"
#include "api.h"
#include "pros/motors.h"
#include "subsystems/subsystems.hpp"
#include "ARMS/config.h"

using namespace pros;

namespace roller {

    namespace{ //Anonymous namespace for private data and methods

        /**
        *
        * PRIVATE DATA
        *
        */

        //The motor used to toggle rollers
        Motor motor(ROLLER_MOTOR, E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_ROTATIONS);
        //The optical sensor on the roller mechanism
        Optical optical(ROLLER_OPTICAL);

        //Target speed
        double speed = 0;
        //State variable: true if currently turning a roller with the automated sensor-driven function
        bool turning_roller = false;
        //Speed for automated sensor-driven function
        int roller_turning_speed = 80;
        //The last task loop's state variable for color (true = red)
        bool last_hue = false;

        /**
        *
        * PRIVATE METHODS
        *
        */

        /**
        * Whether or not the optical sensor sees red.
        *
        * @return True if red, False if not.
        */
        bool isRed() {
            double color = optical.get_hue();
            if (color > 180) {
                return (color - 360 > -30);
            } else {
                return color < 30;
            }
        }

        /**
        * Roller asynchronous task loop function
        */
        void task_func() {
            while (true) {
                if (turning_roller) {
                    motor.move(roller_turning_speed);
                    if (last_hue ^ isRed()) {
                        turning_roller = false;
                        motor.move(-20);
                    }
                } else {
                    motor.move_voltage(120 * speed);
                }
                pros::delay(10);
            }
        }
    }
    
    /**
    *
    * PUBLIC METHODS (see header file for documentation)
    *
    */

    void toggle_turn_roller() {
        turning_roller = !turning_roller;
        last_hue = isRed();
    }

    void move(double speed) {
        roller::speed = speed;
        // printf("\nRoller speed: %f\n", speed);
        // printf("\nBool thing: %d\n", turning_roller);
    }

    void init() {
        motor.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
        optical.set_led_pwm(100);
        pros::Task roller_task(task_func);
    }
} 