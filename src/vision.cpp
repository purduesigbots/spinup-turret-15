#include "vision.h"
#include "subsystems.h"
#include "comms/comms.hpp"

namespace vision {
comms::ReceiveComms communication(8, 115200, START_CHAR, END_CHAR);
targeting_data latency_positions[LATENCY_FRAMES];

double horiz_angle_to_target(int x) {
    return (x - GOAL_CENTER_X) * HORIZ_PIXEL_TO_DEG;
}

double vert_angle_to_target(int y) {
    return -(y - 208) * VERT_PIXEL_TO_DEG;
}

double distance_to_goal(int y) {
    return (TARGET_HEIGHT - CAMERA_HEIGHT)/tanf((CAMERA_ANGLE + vert_angle_to_target(y)) * M_PI / 180);
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
        u_int64_t newcolor = communication.get_data(GOAL_COLOR);
        u_int64_t newlr = communication.get_data(LEFT_RIGHT);
        u_int64_t newheight = communication.get_data(HEIGHT);
        bool updated = (newcolor != color || newlr != lr || newheight != height);

        if (updated) {
            color = newcolor;
            lr = newlr;
            height = newheight;
        }
        
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

        double angle_to_target, dist_to_goal;

        if (updated) {
            angle_to_target = horiz_angle_to_target(lr) + current_position.turret_ang;
            dist_to_goal = distance_to_goal(height);
        } else {
            angle_to_target = current_position.target_ang;
            dist_to_goal = current_position.target_dis;
        }

        double angle_to_aim;
        double third_side_mag = dist_to_goal;

        if (dist_to_goal != 0) {
            arms::Point target;
            target.x = dist_to_goal * cos(angle_to_target * M_PI / 180);
            target.y = dist_to_goal * sin(angle_to_target * M_PI / 180);

            double angle_between_vectors, angle_to_drive;
            double movement_mag = arms::length(current_position.point);
            if (movement_mag != 0) {
                if (current_position.point.x >= 0) {
                    angle_to_drive = 0;
                } else {
                    angle_to_drive = -180;
                }
                double angle_between_calc;
                if (angle_to_target * angle_to_drive >= 0) {
                    angle_between_calc = fabs(angle_to_target - angle_to_drive);
                } else {
                    angle_between_calc = fabs(angle_to_target) + fabs(angle_to_drive);
                }
                if (angle_between_calc > 180) {
                    angle_between_vectors = 360 - angle_between_calc;
                } else {
                    angle_between_vectors = angle_between_calc;
                }
                third_side_mag = sqrtf(powf(current_position.point.x - target.x, 2) + powf(target.y, 2));
            } else {
                angle_between_vectors = 0;
            }
            double angle_to_adjust;
            if (angle_between_vectors != 0) {
                if (angle_between_vectors > 90) {
                    angle_to_adjust = (asin((dist_to_goal * sin(angle_between_vectors * M_PI / 180))/ third_side_mag)) * 180/M_PI;
                } else {
                    angle_to_adjust = 180 - (asin((dist_to_goal * sin(angle_between_vectors * M_PI / 180))/ third_side_mag)) * 180/M_PI;
                }
            }
            double cross_product = current_position.point.x * target.y;

            if (movement_mag != 0 && angle_to_adjust != 0) {
                if (current_position.point.x >= 0) {
                    if (cross_product >= 0) {
                        angle_to_aim = 180 - angle_to_adjust;
                    } else {
                        angle_to_aim = -180 + angle_to_adjust;
                    }
                } else {
                    if (cross_product >= 0) {
                        angle_to_aim = -angle_to_adjust;
                    } else {
                        angle_to_aim = angle_to_adjust;
                    }
                }
            } else {
                angle_to_aim = angle_to_target;
            }
        }
        double final_angle = angle_to_aim - current_position.theta;
        turret::speed = (final_angle - turret::target_angle) * FPS * 60 / 360;
        turret::target_angle = final_angle;

        for (int n = 0; n < LATENCY_FRAMES - 1; n++) {
            latency_positions[n] = latency_positions[n+1];
        }

        latency_positions[LATENCY_FRAMES-1].point = {0,0};
        latency_positions[LATENCY_FRAMES-1].target_ang = final_angle;
        latency_positions[LATENCY_FRAMES-1].turret_ang = turret::current_angle;
        latency_positions[LATENCY_FRAMES-1].target_dis = third_side_mag;
    }
}
}