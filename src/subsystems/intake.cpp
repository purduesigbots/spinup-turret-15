#include "main.h"
#include "math.h"
#include "pros/adi.hpp"
#include "pros/misc.hpp"
#include "pros/motors.h"
#include "subsystems/subsystems.hpp"
#include <cmath>

#include "subsystems/flywheel.hpp"

using namespace pros;

namespace intake {

Motor left_motor(11, E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_ROTATIONS);
Motor right_motor(19, E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_ROTATIONS);
ADIAnalogIn line(6);

bool running = false;
double speed = 0;

// Data for the intake's arm. The arm should start raised for every match.
ADIDigitalOut armPiston({{8, 'a'}});
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

int expect(int numDiscs, int timeout) {
    // TODO: implement
}

bool is_on() {
    return running;
}

void raise_arm() {
    if(!armRaised) {
        armRaised = true;
        armState = !armState;
        armPiston.set_value(armState);
    }
}

void lower_arm() {
    if(armRaised) {
        armState = !armState;
        armRaised = false;
    }
}

void toggle_arm() {
    if(armRaised) {
        lower_arm();
    }
    else {
        raise_arm();
    }
}

bool arm_raised() {
    return armRaised;
}

// This probably needs rewritten:
bool clear() {
    return line.get_value() > 1500;
}

} // namespace intake
