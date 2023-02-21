#pragma once

#include "api.h"

#if 0
/**
 * Subsystem Dependencies: Disc Lift
 */
namespace intake {
    /**
     * Starts the intake of the robot with a specified speed
     * 
     * @param speed RPM to run the intake at
     */
    void start(double speed);

    /**
     * Toggles the intake of the robot
     *
     * @param speed RPM to run the intake at, if it is toggled on
     * @param brakeMode The brakemode to use if stopping
     */
    bool toggle(double speed, pros::brake_mode_e brakeMode = pros::E_BRAKE_STOP);

    /**
     * Tells the halt until a certain number of discs are taken up.
     * 
     * This is intended for auton programming. This allows the program to wait
     * until a number of discs are picked up, but give up 
     * 
     * @param numDiscs How many discs to expect/pick up
     * @param timeout  How long to try before giving up
     * 
     * @return The number of discs that were detected when picking up
     */
    int expect(int numDiscs, int timeout = 5000);

    /**
     * Stops the intake with a specified brake mode
     * 
     * @param brakeMode The brakemode to use
     */
    void stop(pros::brake_mode_e brakeMode = pros::E_BRAKE_STOP);

    /**
     * Returns whether the intake is on or not
     */
    bool is_on();
}

#endif


namespace intake {
extern pros::Motor left_motor;
extern pros::Motor right_motor;
extern double speed;
void move(double speed);
void toggle();
bool clear();
} // namespace intake