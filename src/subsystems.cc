#include "main.h"
#include "pros/misc.hpp"
#include "pros/motors.h"
#include "subsystems.h"

// intake -------------------------------------------------------------------------
namespace intake {

Motor left_motor(12, MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_ROTATIONS);
Motor right_motor(20, MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_ROTATIONS);

double speed = 0;

void move(double speed) {
    left_motor.move_voltage(120 * speed);
    right_motor.move_voltage(120 * speed);
    intake::speed = speed;
}

} // intake

// roller -------------------------------------------------------------------------
namespace roller {

Motor motor(7, MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_ROTATIONS);
Optical optical(4);

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
}

void task() {
    while (true) {
        lcd::set_text(0, "Optical: " + std::to_string(optical.get_hue()));
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

} // roller

// turret -------------------------------------------------------------------------
namespace turret {

Motor motor(5, MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_ROTATIONS);

double speed = 0;

void move(double speed) {
    motor.move_voltage(120 * speed);
    turret::speed = speed;
}

} // turret
