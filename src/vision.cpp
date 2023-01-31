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
arms::Point prev_point;
double prev_heading;

void task() {
    communication.start();
    prev_point = arms::odom::getPosition();
    prev_heading = arms::odom::getHeading();
    while (true) {
        // update information from camera
        color = communication.get_data(GOAL_COLOR);
        lr = communication.get_data(LEFT_RIGHT);
        height = communication.get_data(HEIGHT);
        
        // update positions of latency frames
        for (int i = 0; i < FRAME_TIMING/ODOM_UPDATE_SPEED; i++) {
            pros::delay(ODOM_UPDATE_SPEED);
            arms::Point point_diff = arms::odom::getPosition() - prev_point;
            double heading_diff = arms::odom::getHeading() - prev_heading;
            for (int j = 0; j < LATENCY_FRAMES; j++) {
                latency_positions[j].point += point_diff;
                latency_positions[j].theta += heading_diff;
            }
        }

        targeting_data current_position = latency_positions[0];

        double angle_to_target = horiz_angle_to_target(lr);
    }
}
}