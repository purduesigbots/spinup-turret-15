// clang-format off
#include "vision.h"
#include "subsystems.h"
#include "subsystems/subsystems.hpp"
#include "comms/comms.hpp"
#include "ARMS/odom.h"
#include "pros/misc.h"
#include "LatPullDown/Oak_1_latency_compensator.hpp"
// clang-format on
#if BOT == SILVER
    #include "../include/ARMS/config_silver.h"
#elif BOT == GOLD
    #include "../include/ARMS/config_gold.h"
#endif
#define START_CHAR 0b11001100
#define END_CHAR 0b00110011

#define GOAL_COLOR 0b00000001
#define LEFT_RIGHT 0b00000010
#define HEIGHT 0b00000011
#define WIDTH 0b00000100

const std::tuple<double, double> GOAL_POS = {50 * sqrt(2), 0};

const double IMAGE_HEIGHT = 416;
const double GOAL_HEIGHT = 13.87;
const double GOAL_WIDTH = 16;
const double FOCAL_LENGTH = 0.5;

// Important, this is the size of the physical sensor, not its height off the
// ground!
// https://stackoverflow.com/questions/50125574/calculate-image-size-of-an-object-from-real-size
const double SENSOR_HEIGHT = 0.3074803;

namespace vision {

std::shared_ptr<comms::ReceiveComms> communication;

void init() {
  communication =
      std::make_shared<comms::ReceiveComms>(IRIS_PORT, 115200, START_CHAR, END_CHAR);
}

double get_goal_gamma() {
  return atan2((GOAL_WIDTH / (double)communication->get_data(WIDTH) *
                (220 - (double)communication->get_data(LEFT_RIGHT))),
               get_goal_distance()) *
         (180 * M_1_PI);
}

double get_goal_distance() {
  return (FOCAL_LENGTH * GOAL_HEIGHT * IMAGE_HEIGHT) /
         (communication->get_data(HEIGHT) * SENSOR_HEIGHT);
}

std::tuple<double, double, double> get_turret_pose() {
  // get odom x,y, and heading
  // get turret heading
  arms::Point p = arms::odom::getPosition();
  //-7.5
  // 7.2
  return std::make_tuple(p.x, p.y,
                         arms::odom::getHeading() + turret::get_angle() +
                             get_goal_gamma());
}

double get_latency() {
  return 40; // 40ms latency
}

Oak_1_latency_compensator *latency_compensator;

void task() {
  communication->start();

  latency_compensator = new Oak_1_latency_compensator(
      7,                 // max buffer of 7
      10,                // get odom position every 10ms
      get_turret_pose,   // function to get pose of the turret
      get_goal_distance, // function to calulate the vector to the goal based on
                         // the distance
      get_latency,       // function to get the latency of the current frame
      false);

  float previous_speed = 0;

  float previous_height;
  float previous_lr;
  float previous_color;
  float counter = 0;

  while (true) {
    uint64_t color = communication->get_data(GOAL_COLOR);
    uint64_t lr = communication->get_data(LEFT_RIGHT);
    uint64_t height = communication->get_data(HEIGHT);
    uint64_t width = communication->get_data(WIDTH);

    double turn_degrees =
        latency_compensator->get_new_goal_distance(get_goal_gamma());
    if (color == 0) {
      turn_degrees = 0;
    }

    // printf("Color:      %llu\n", color);
    // printf("Left/Right: %llu\n", lr);
    // printf("Height:     %llu\n", height);
    // printf("Width:      %llu\n", width);
    // printf("Gamma:      %f\n", get_goal_gamma());
    // printf("Turn Degrees: %f\n", turn_degrees);
    // printf("Goal Distance: %f\n", get_goal_distance());
    // printf("Turrent Angle %f\n", turret::get_angle());
    // printf("Turrent Pose %f\n", std::get<2>(get_turret_pose()));
    // printf("Time:       %f\n", counter);

    if (previous_height != height || previous_lr != lr ||
        previous_color != color) {
      counter = 0;
    } else {
      counter += 10;
    }

    previous_height = height;
    previous_lr = lr;
    previous_color = color;
    //printf("----------------\n");

    // 0 is red
    // 1 is blue
    // 3 is nothing detected

    if (turret::get_angle() >= 74) {
      turn_degrees = std::min(turn_degrees, 0.0);
    } else if (turret::get_angle() <= -74) {
      turn_degrees = std::max(turn_degrees, 0.0);
    }

    float deadzone = 10.0;

    if (fabs(lr - 265) <= deadzone) {
      turn_degrees = 0.0;
    }

    //turret::move(turn_degrees * 7.5);

    pros::delay(10);
  }
}
} // namespace vision