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
     * Returns the aim error of the vision system. Positive values 
     * mean the turret needs to adjust to the left, 
     * negative values mean the turret needs to adjust to the right.
     * 
     * @return The aim error of the vision system
     */
    double get_error(bool radians = false);

    /**
    * Sets the goal targetting mode (color).
    *
    * @param targ The goal color to target (RED, BLUE, or BOTH)
    */
    void set_targ_goal(Goal targ);

    /**
    * Gets the goal targetting mode (color).
    *
    * @return The goal color to target (RED, BLUE, or BOTH)
    */
    Goal get_targ_goal();

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

     /**
    * Gets the goal position of the target goal seen by vision
    *
    * @return The goal position in inches
    */
    arms::Point get_goal_pos();
}

