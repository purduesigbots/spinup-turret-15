#ifndef _VISION_H_
#define _VISION_H_

#include "ARMS/api.h"

/* Camera Constants */
#define FPS 30.0 // FPS of the camera
#define FRAME_TIMING (1000.0/FPS)
#define GOAL_CENTER_X 260 // X coordinate for the goal to be centered.
#define HORIZ_PIXEL_TO_DEG 0.10 // Constant to convert difference in x coordinate to difference in degrees. Calculated as horizontal FOV / horizontal resolution.
#define VERT_PIXEL_TO_DEG 0.12
#define ODOM_UPDATE_SPEED 10.0 // Speed in ms of the odom update loop
#define LATENCY_FRAMES 8
#define TARGET_HEIGHT 21
#define CAMERA_HEIGHT 10
#define CAMERA_ANGLE 3

/* Communications Constants */
#define START_CHAR 0b11001100
#define END_CHAR 0b00110011

#define GOAL_COLOR 0b00000001
#define LEFT_RIGHT 0b00000010
#define HEIGHT 0b00000011

namespace vision {
    struct targeting_data {
        arms::Point point;
        double theta;
        double turret_ang;
        double target_ang;
        double target_dis;
    };
    extern uint64_t color;
    extern uint64_t lr;
    extern uint64_t height;
}

#endif