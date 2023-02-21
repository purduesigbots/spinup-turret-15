#pragma once

#include "api.h"

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
 * Stops the intake with a specified brake mode
 */
void stop();

/**
 * Toggles the intake of the robot
 *
 * @param speed RPM to run the intake at, if it is toggled on
 */
void toggle(double speed);

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
 * Returns whether the intake is on or not
 */
bool is_on();

/**
 * Raises the intake's arm 
 */
void raise_arm();

/**
 * Lowers the intake's arm
 */
void lower_arm();

/**
 * Toggles the intake's arm 
 */
void toggle_arm();

/**
 * Returns whether the intake's arm is raised or not 
 *
 * @return True if the intake's arm is raised
 */
bool arm_raised();

bool clear();

} // namespace intake
