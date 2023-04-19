#pragma once
#include "api.h"

/**
* The disc lift subsystem is responsible for moving the disc lift up and down in order to
* load the turret with discs.
*/
namespace discLift {

    /**
     * Raises the disc lift.
     */
    void discLiftUp();

    /**
     * Holds the disc lift (for firing).
     */
    void discLiftHold();

    /**
     * Lowers the disc lift.
     */
    void discLiftDown();

    /**
     * Homes the disc lift's position.
     *
     * This is blocking unless called in a task.
     */
    void home();

    int disc_count();

    /**
    * Renders the debug screen to the LLEMU display 
    */
    void debug_screen();

}