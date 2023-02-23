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

    void initialize();

    /**
     * Calibrates the turret.
     * 
     * Calibrates the turret by turning it until it hits the bumber switch, then
     * sends it back to face straight forward. The turret's position is then
     * reset and coordinate system is established.
     */
    void calibrate();

    void update();

    void goto_angle(double angle, double velocity = 100, bool async = false);

    void toggle_vision_aim();

    /**
     * Returns the angle of the turret in degrees, where 0 is directly forward,
     * 90 is 90 degrees to the left, and -90 is 90 degrees to the right
     */
    double get_angle();

    bool settled();

    void wait_until_settled();
}