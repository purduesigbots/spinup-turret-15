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

//LOCAL DEFS:
int smart_port = 8;
char adi_port = 'a';
ADIDigitalOut intake_piston({{smart_port,adi_port}});
Motor left_motor(11, E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_ROTATIONS);
Motor right_motor(19, E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_ROTATIONS);
ADIAnalogIn line(6);

bool state = false;
double speed = 0;

void move(double speed) {
    left_motor.move_voltage(120 * speed);
    right_motor.move_voltage(120 * speed);
    intake::speed = speed;
}

void toggle(){
    state = !state;
    intake_piston.set_value(state);
}

bool clear() {
    return line.get_value() > 1500;
}
} // namespace intake