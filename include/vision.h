#ifndef _VISION_H_
#define _VISION_H_

#include "comms/comms.hpp"

namespace vision {
    void init();
    void task();
    extern std::shared_ptr<comms::ReceiveComms> communication;

    double get_goal_distance();
    double get_goal_gamma();
    bool vision_not_working();
    void set_vision_offset(bool is_auto);
    void start_vision();
    int get_goal_color();
}

#endif