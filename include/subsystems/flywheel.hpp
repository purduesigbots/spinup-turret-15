#pragma once

#include "api.h"
#include <stdint.h>
#include <vector>

/**
 * Subsystem Dependencies: Disc Lift
 */
namespace flywheel {

/**
 * Initializes the flywheel subsystem.
 */
void initialize();

/**
 * Starts the flywheel with a specified RPM.
 *
 * @param targetSpeed The RPM that the flywheel should target
 */
void start(double targetSpeed);

/**
 * Sets the speed that the flywheel should target.
 *
 * This is intended to be used when the flywheel is already running
 *
 * @param targetSpeed The RPM that the flywheel should target
 */
void set_target_speed(double targetSpeed);

/**
 * Changes the target speed of the flywheel by a specified amount.
 *
 * This amount is relative to the flywheel's current speed.
 *
 * @param amount How much to change the flywheel's target RPM by
 */
void change_target_speed(double amount);

/**
 * Tells the flywheel to coast to a stop
 */
void stop();

/**
 * Toggles the flywheel on and off.
 *
 * @param targetSpeed If the flywheel is toggled on, this is the RPM that it
 *                    should target
 */

void toggle(double targetSpeed);

/**
 * Blocks the calling thread until the flywheel reaches its target speed, or the
 * timeout is reached.
 *
 * @return True if the timeout was reached.
 */
bool wait_until_at_speed(uint32_t timeout);

bool wait_until_fired(uint32_t timeout);

/**
 * Returns whether the flywheel is at its target RPM or not
 *
 * @return Whether the flywheel has reached its target RPM or not
 */
bool at_speed();

/**
 * Gets the current RPM that the flywheel is running at
 *
 * @return Current RPM of the flywheel
 */
double current_speed();

/**
 * Gets the current RPM of one of the flywheel motors
 *
 * @return Current RPM of the flywheel
 */
double current_speed(int n);

/**
 * Gets the target RPM that the flywheel is trying to reach
 *
 * @return The target RPM that the flywheel is trying to reach
 */
double target_speed();

/**
 * Shoots a specified number of discs.
 *
 * Attempts to shoot a specified number of discs within a given time
 * period. If the timeout period is reached, the function will return
 * regardless of the number of discs fired
 *
 * @param numDiscs The number of discs to try and fire
 * @param timeout  The number of milliseconds to try to fire the specified
 *                 number of discs. If this is less than 0, there is no
 *                 timeout and the robot will keep trying until it
 *                 successfully fires all the discs.
 *
 * @return The number of discs successfully shot before the timeout was
 *         reached
 */
int fire(int numDiscs = 1, int timeout = 5000);

// Fires a number of discs, each with a specified speed. This function takes
// a vector of doubles where each double is a speed for the disk. The size
// of the passed vector specifies how many discs are shot.
// int fire(std::vector<double> speeds, int timeout = 5000);

void fireControl_driver(bool enable);

// sets the indexer to firing mode if enable = true, and to 0 if enable = false.

void debug_screen();

} // namespace flywheel