#pragma once

#include "api.h"

/**
 * Turret Subsystem: Interface to controll the aiming of the turret. 
 * 
 * Coodinate System: After calibration, the turret should face forward, which is
 * 0.0. Then 90 will be 90 degrees to the left, and -90 will be 90 degrees to
 * the right.
 * 
 * Subsystem Dependencies: None
 */
namespace turret {
extern pros::Motor motor;

/**
 * Initilalize the turret subsystem. This also calibrates the turret
 */
void initialize();

/**
 * Calibrates the turret.
 * 
 * Calibrates the turret by turning it until it hits the bumber switch, then
 * sends it back to face straight forward. The turret's position is then
 * reset and coordinate system is established.
 */
void calibrate();


/**
 * Tells the turret to go to an angle relative to the robot's heading
 * 
 * -90 is 90 degrees to the left, 90 is 90 degrees to the right. 0 is 
 * straight forwards with the robot's heading
 * 
 * @param angle The angle to face
 * @param velocity The velocity to move with
 * @param async If true, this function will not block the calling thread
 */
void goto_angle(double angle, double velocity = 100, bool async = false);

/**
 * Tells the turret to go to an angle relative to the robot's heading and
 * the current turret angle
 * 
 * @param angle The angle to move by in degrees
 * @param velocity The velocity to move with.
 * @param async If strue, this function will not block the calling thread
 */
void goto_rel_move(double angle, double velocity = 100, bool async = false);

/**
 * Returns the angle of the turret in degrees, where 0 is directly forward,
 * 90 is 90 degrees to the left, and -90 is 90 degrees to the right
 */
double get_angle();

/**
 * Returns whether the turret has reached its target angle
 * 
 * @return True if the turret has reached its target angle
 */
bool settled();

/**
 * Blocks the calling thread until the turret reaches its target angle 
 */
void wait_until_settled();


void toggle_vision_aim();
void enable_vision_aim();
void disable_vision_aim();

/**
 * 
 */
void debug_screen();

}