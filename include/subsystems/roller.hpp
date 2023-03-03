#pragma once

#include "api.h"

namespace roller {

extern void init();
extern pros::Motor motor;
extern pros::Optical optical;
extern double speed;
void move(double speed);
void toggle_turn_roller();
void set_brake_mode(pros::motor_brake_mode_e mode);

} // namespace roller