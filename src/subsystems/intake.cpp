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

bool running = false;
double speed = 0;

// Data for the intake's arm. The arm should start raised for every match.
ADIDigitalOut armPiston({{8, 'a'}});
bool armRaised = true;
bool armState = false;

void disc_counting_task_function(void* data) {
    while(true) {

    }
}

Task discCountingTask(disc_counting_task_function);

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
    int startTime = pros::millis();
    int startNumDiscs = disccounter::disc_count();

    while(
        disccounter::disc_count() - startNumDiscs < numDiscs &&
        pros::millis() - startTime < timeout
    ) {
        pros::delay(10);
    }

    pros::delay(500);
    
    return disccounter::disc_count() - startNumDiscs;
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

} // namespace intake
