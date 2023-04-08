#pragma once

#include "api.h"

/**
 * The intake subsystem is responsible for intaking discs.
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
    * @param speed RPM to run the intake at, if it is being toggled on
    * otherwise, the speed parameter is ignored.
    */
    void toggle(double speed);

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
} 
