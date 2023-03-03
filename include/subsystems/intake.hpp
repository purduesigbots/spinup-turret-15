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

/**
 * Returns whether the intake is intaking or not.
 * 
 * @return True if the intake is intaking. 
 */
bool intaking();

/**
 * Returns whether the intake is outtaking or not, i.e., running in reverse
 * 
 * @return True if the intake is outtaking.
 */
bool outtaking();


} // namespace intake
