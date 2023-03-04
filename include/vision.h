#ifndef _VISION_H_
#define _VISION_H_

namespace vision {
    void init();
    void task();

    double get_goal_distance();
    double get_goal_gamma();
    bool vision_not_working();
    /**
    *  Sets the offset of the aimbot. Higher = more left offset.
    *
    * @param offset amount to offset the aimbot
    */
    void set_vision_offset(int offset);
    void start_vision();
}

#endif