#include "vision.h"
#include "comms/comms.hpp"

namespace vision {
comms::ReceiveComms communication(8, 115200, START_CHAR, END_CHAR);
targeting_data latency_positions[LATENCY_FRAMES];

double horiz_angle_to_target(double x) {
    return (x - GOAL_CENTER_X) * HORIZ_PIXEL_TO_DEG;
}

/* 0: blue, 1: red */
u_int64_t color = 0;
/* 0-416 left-right */
u_int64_t lr = 0;
u_int64_t height = 0;

void task() {
    communication.start();
    while (true) {
        color = communication.get_data(GOAL_COLOR);
        lr = communication.get_data(LEFT_RIGHT);
        height = communication.get_data(HEIGHT);
        
        for (int i = 0; i < FRAME_TIMING/ODOM_UPDATE_SPEED; i++) {
            
        }
    }
}
}