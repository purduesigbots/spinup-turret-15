#ifndef _VISION_H_
#define _VISION_H_

namespace vision {
    /**
    *  Initializes the vision system.
    */
    void init();

    /**
    * Task function
    */
    void task();

    /**
    *  Gets the distance to the goal.
    *
    * @return distance to the goal in inches
    */
    double get_goal_distance();

    /**
    *  Gets the angle to the goal.
    *
    * @return angle error in degrees
    */
    double get_goal_gamma();

    /**
    *  Gets the status of the vision system.
    *
    * @return whether or not the vision system is operating correctly.
    */
    bool vision_not_working();

    /**
    *  Sets the offset of the aimbot. Higher = more left offset.
    *
    * @param offset amount to offset the aimbot
    */
    void set_vision_offset(int offset);

    /**
    *  Starts the vision system.
    */
    void start_vision();

    /**
    * Gets the color of the goal seen most recently.
    * 
    * @return 0 if red, 1 if blue
    */
    int get_goal_color();

    /**
    * Whetehr or not the camera sees a goal
    *
    * @return true if a goal is currently in view
    */
    bool get_goal_detected();
}

#endif