#pragma once

/**
 * The disc counting subsystem is in charge of keeping track of when discs enter
 * or leave the robot. 
 */
namespace disccounter {

/**
 * Initializes the disc counter subsystem: Creates tasks, sets defaults, etc. 
 */
void initialize();

/**
 * Returns true if the robot is currently seeing a disc.
*/
bool seeing_disc();

/**
 * Renders the debug screen to the LVGL display 
 */
void debug_screen();

/**
 * Returns the number of discs currently in the robot 
 */
int disc_count();

/**
 * Decrements the number of discs currently in the robot. 
 * 
 * This here solely to be used by the turret subsystem, that way it can
 * decrement the number of discs in the robot when it fires one off.  
 * 
 * Please do not use this anywhere else!!!1
 */
void decrement();

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

} // namespace disccounter