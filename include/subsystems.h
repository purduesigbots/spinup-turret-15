#ifndef _SUBSYSTEMS_H_
#define _SUBSYSTEMS_H_

#include "api.h"
#include "pros/motors.h"

using namespace pros;




namespace disklift {
    extern Motor lift_motor;
    void move(double speed);
    void move_to(double position,double speed);
    void toggle_move();
}





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
void move(double speed);


} // namespace turret

namespace flywheel {
extern sylib::Motor flywheel1;
extern sylib::Motor flywheel2;
extern double speed;
void move(double speed);
bool at_speed();
void wait_until_fired();
void wait_until_at_speed();
void task();
}

#endif