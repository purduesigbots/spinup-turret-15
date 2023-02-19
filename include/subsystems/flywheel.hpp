#pragma once 

#if 0
/**
 * Subsystem Dependencies: Disc Lift
 */
namespace flywheel {
    // Starts the flywheel and tells it to target a specific speed
    void start(double targetSpeed);

    // Tells the flywheel to target a certain speed. 
    void set_target(double targetSpeed);

    void stop();

    // Blocks until the robot reaches a specific speed.
    void wait_until_at_speed();

    bool at_speed();

    // Shoots a specified number of discs. Returns the number of discs that were
    // actually shot. Will return once all numDiscs are shot, or the timeout
    // period is reached. 
    int fire(int numDiscs, int timeout = 5000);

    // Fires a number of discs, each with a specified speed. This function takes
    // a vector of doubles where each double is a speed for the disk. The size
    // of the passed vector specifies how many discs are shot. 
    int fire(std::vector<double> speeds, int timeout = 5000);
}

#endif