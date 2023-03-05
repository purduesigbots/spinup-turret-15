#include "main.h"
#include "math.h"
#include "pros/adi.hpp"
#include "pros/misc.hpp"
#include "pros/motors.h"
#include "subsystems/subsystems.hpp"
#include <cmath>

#include "subsystems/flywheel.hpp"

#include "ARMS/config.h"

using namespace pros;

namespace intake {

Motor left_motor(INTAKE_LEFT, E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_ROTATIONS);
Motor right_motor(INTAKE_RIGHT, E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_ROTATIONS);

bool running = false;
double speed = 0;

// Data for the intake's arm. The arm should start raised for every match.
ADIDigitalOut armPiston(INTAKE_PISTON);
bool armRaised = true;
bool armState = false;

void start(double speed) {
    left_motor.move_voltage(120 * speed);
    right_motor.move_voltage(120 * speed);
    intake::speed = speed;
}

void stop() {
    left_motor.move_voltage(0);
    right_motor.move_voltage(0);
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

bool is_on() {
    return running;
}

void raise_arm() {
    if(!armRaised) {
        toggle_arm();
    }
}

void lower_arm() {
    if(armRaised) {
        toggle_arm();
    }
}

void toggle_arm() {
    armState = !armState;
    armRaised = !armRaised;
    armPiston.set_value(armState);
}

bool arm_raised() {
    return armRaised;
}

bool intaking() {
    return speed > 0.0;
}

bool outtaking() {
    return speed < 0.0;
}

} // namespace intake
