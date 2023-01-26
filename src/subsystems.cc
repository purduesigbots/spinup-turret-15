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

double speed = 0;

void move(double speed) {
    motor.move_voltage(120 * speed);
    roller::speed = speed;
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
