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
extern void home();
} // namespace disklift





namespace intake {
extern Motor left_motor;
extern Motor right_motor;
extern double speed;
void move(double speed);
void toggle();
bool clear();
} // namespace intake

namespace roller {
extern void init();
extern Motor motor;
extern Optical optical;
extern double speed;
void move(double speed);
void toggle_turn_roller();
void set_brake_mode(pros::motor_brake_mode_e mode);
} // namespace roller


//deflector__________________________________________________________
namespace deflector {
void toggle();
}

extern bool isSilva();

#endif