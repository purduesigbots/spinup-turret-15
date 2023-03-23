#pragma once

#include "ARMS/api.h"

namespace vision {
    
    /**
    * Enumerated class containing the color of the goal
    *
    * RED: Red goal
    *
    * BLUE: Blue goal
    *
    * BOTH: Both goals
    */
    enum class Goal {
        RED,
        BLUE,
        BOTH
    };

    /**
     * Initializes the vision system.
     */
    void init();    
    
    /**
     * Returns the aim error of the vision system.
     * 
     * @return The aim error of the vision system
     */
    int get_error();

    /**
    * Sets the goal targetting mode (color).
    *
    * @param targ The goal color to target (RED, BLUE, or BOTH)
    */
    void set_targ_color(Goal targ);

    /**
    * Gets whether or not the vision system is working
    *
    * @return true if vision is working, false if not
    */
    bool is_working();

    /**
    * Gets the distance FROM THE CAMERA LENS to the gal
    *
    * @return The distance to the goal in inches. Returns -1 
    * if there's no goal saved or in view.
    */
    double get_distance();
}

