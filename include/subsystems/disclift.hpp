#pragma once

#include "api.h"

namespace disclift {

extern pros::Motor lift_motor;
extern void discLiftUp();
extern void discLiftHold();
extern void discLiftDown();
extern void calculatePos();
extern void home();

} // namespace disklift