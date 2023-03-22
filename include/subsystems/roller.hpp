#pragma once

#include "api.h"

/**
* The roller subsystem is responsible for toggling the rollers.
*/
namespace roller {

    /**
    * Initializes the roller mech
    */
    void init();

    /**
    * Moves the roller mech at a specified speed
    *
    * @param speed The RPM to move the roller mech at.
    */
    void move(double speed);

    /**
    * Toggles a roller (based on color sensor)
    */
    void toggle_turn_roller();
}