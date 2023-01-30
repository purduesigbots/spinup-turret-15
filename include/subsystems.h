#ifndef _SUBSYSTEMS_H_
#define _SUBSYSTEMS_H_

#include "api.h"

using namespace pros;

namespace intake {
extern Motor left_motor;
extern Motor right_motor;
extern double speed;
void move(double speed);
} // namespace intake

namespace roller {
extern Motor motor;
extern Optical optical;
extern double speed;
void move(double speed);
void toggle_turn_roller();
void init();
} // namespace roller

namespace turret {
extern Motor motor;
extern double speed;
void move(double speed);
} // namespace turret

#endif