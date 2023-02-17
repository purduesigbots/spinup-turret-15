#pragma once

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
    /**
     * Calibrates the turret.
     * 
     * Calibrates the turret by turning it until it hits the bumber switch, then
     * sends it back to face straight forward. The turret's position is then
     * reset and coordinate system is established.
     */
    void calibrate();

    /**
     * Sends the turret to 0.0
     * 
     * @param async Tells the function to wait asyncronously
     */
    void home(bool async);

    /**
     * Tells the turret to move relative to it's current angle.
     * 
     * @param angle Degrees to turn, relative to current position
     * @param 
     */
    void goto_angle_rel(double pos, double vel, bool async = false);

    /**
     * Tells the turret to go to an angle in the global (field) coordinate
     * system. If the robot cannot turn to face the angle, it will get as close
     * as it can. 
     */
    void goto_angle_glob(double pos);

    void turn_angle_rel();

    void turn_angle_glob();

    /**
     * Blocks execution until the turret reaches the point it is supposed to. 
     */
    void wait_until_setteld();
}