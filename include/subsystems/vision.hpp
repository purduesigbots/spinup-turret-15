#pragma once

#include "ARMS/api.h"

namespace vision {
    /**
     * Initializes the vision system.
     */
    void init();

    /**
     * Gets the distance to the goal.
     *
     * @return distance to the goal in inches
     */
    double get_goal_distance();

    /**
    * Gets the vision's estimate of the location of the goal.
    *
    * @return a point representing the goal's position.
    */
    arms::Point get_goal_point();

    /**
    * Gets the angle to the goal.
    *
    * @return angle to the goal in degrees
    */
    double get_goal_point_gamma();

    /**
    * Gets the distance to the goal.
    *
    * @return distance to the goal in inches
    */
    double get_goal_point_distance();

    /** 
     * Gets the status of the vision system.
     *
     * @return whether or not the vision system is operating correctly.
     */
     bool vision_not_working();

    /**
     * Sets the offset of the aimbot. Higher = more left offset.
     *
     * @param offset amount to offset the aimbot
     */
    void set_vision_offset(int offset);

    /**
     * Gets the color of the goal seen most recently.
     * 
     * @return 0 if red, 1 if blue
     */
    int get_goal_color();

    /**
     * Whether or not the camera sees a goal
     *
     * @return true if a goal is currently in view
     */
    bool get_goal_detected();
}

