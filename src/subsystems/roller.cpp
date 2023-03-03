#include "subsystems/roller.hpp"
#include "main.h"
#include "api.h"
#include "subsystems/subsystems.hpp"

#include "ARMS/config.h"

using namespace pros;

namespace roller {

//LOCAL DEFS:
// ^^^ These aren't very local since they externed in the header
Motor motor(ROLLER_MOTOR, E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_ROTATIONS);
Optical optical(5);

double speed = 0;
bool turning_roller = false;
int roller_turning_speed = 80;
bool last_hue = false;

bool isRed() {
    double color = optical.get_hue();
    if (color > 180) {
        return (color - 360 > -30);
    } else {
        return color < 30;
    }
}

void toggle_turn_roller() {
    turning_roller = !turning_roller;
    last_hue = isRed();
}

void move(double speed) {
    roller::speed = speed;
    printf("\nRoller speed: %f\n", speed);
    printf("\nBool thing: %d\n", turning_roller);
}

void task() {
    while (true) {
        if (turning_roller) {
            motor.move(roller_turning_speed);
            if (last_hue ^ isRed()) {
                turning_roller = false;
                motor.move(-20);
            }
        } else {
            motor.move(120 * speed);
        }
        pros::delay(10);
    }
}

void init() {
    optical.set_led_pwm(100);
    pros::Task roller_task(task);
}

void set_brake_mode(pros::motor_brake_mode_e mode) {
    motor.set_brake_mode(mode);
}

} // namespace roller