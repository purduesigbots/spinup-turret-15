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

} // namespace disccounter