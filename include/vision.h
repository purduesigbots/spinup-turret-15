#ifndef _VISION_H_
#define _VISION_H_

namespace vision {
    void init();
    void task();

    double get_goal_distance();
    double get_goal_gamma();
    bool vision_not_working();
    void set_vision_offset(bool is_auto);
    void start_vision();
}

#endif