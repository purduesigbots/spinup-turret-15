#ifndef _SUBSYSTEMS_H_
#define _SUBSYSTEMS_H_

#include "main.h"

using namespace pros;




namespace disklift {
extern Motor lift_motor;
extern void discLiftUp();
extern void discLiftHold();
extern void discLiftDown();
extern void calculatePos();
} // namespace disklift





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
extern Motor turret_motor;
extern double speed;
extern double target_angle;
extern double current_angle;
void move(double speed);
void home();
double get_position();
} // namespace turret

namespace flywheel {
extern sylib::Motor left_flywheel;
extern sylib::Motor right_flywheel;
extern pros::Motor indexer;
extern double speed;
void move(double speed);
bool at_speed();
void wait_until_fired();
void wait_until_at_speed();
void fire();
void stopIndexer();
void task();
}

#endif