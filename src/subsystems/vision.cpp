#include "subsystems/subsystems.hpp"
#include "comms/comms.hpp"
#include "LatPullDown/Oak_1_latency_compensator.hpp"
#include "ARMS/config.h"

namespace vision {

  namespace{ //Anonymous namespace for private data and methods

    /**
    *
    * PRIVATE DATA
    *
    */

    //IRIS COMMUNICATIONS CONSTANTS
    #define START_CHAR 0b11001100
    #define END_CHAR 0b00110011
    #define GOAL_COLOR 0b00000001
    #define LEFT_RIGHT 0b00000010
    #define HEIGHT 0b00000011
    #define WIDTH 0b00000100

    //Goal position tuple
    const std::tuple<double, double> GOAL_POS = {50 * sqrt(2), 0};

    //Image constants
    const double IMAGE_HEIGHT = 416;
    const double GOAL_HEIGHT = 13.87;
    const double GOAL_WIDTH = 16;
    const double FOCAL_LENGTH = 0.5;

    // Important, this is the size of the physical sensor, not its height off the
    // ground!
    // https://stackoverflow.com/questions/50125574/calculate-image-size-of-an-object-from-real-size
    const double SENSOR_HEIGHT = 0.3074803;
    std::shared_ptr<comms::ReceiveComms> communication;
    double vision_offset = 240;
    arms::Point goal_point = {0,0};
    int no_new_comm_count = 0;

    uint64_t goal_color = 0;
    uint64_t left_right = 0;
    uint64_t height = 0;
    uint64_t width = 0;

  } //End anonymous namespace


void init() {
  communication =
      std::make_shared<comms::ReceiveComms>(IRIS_PORT, 115200, START_CHAR, END_CHAR);
}

double get_goal_gamma() {
  return atan2((GOAL_WIDTH / (double)width *
                (vision_offset - (double)left_right)),
               get_goal_distance()) *
         (180 * M_1_PI);
}

double get_goal_distance() {
  return (FOCAL_LENGTH * GOAL_HEIGHT * IMAGE_HEIGHT) /
         (height * SENSOR_HEIGHT);
}

arms::Point get_goal_point() {
  return goal_point;
}

double get_goal_point_gamma() {
  arms::Point diff = goal_point - arms::odom::getPosition();
  double gamma = atan2(diff.y, diff.x);
  if (diff.x < 0) {
    gamma += M_PI;
  }
  gamma *= 180 * M_1_PI;
  gamma -= arms::odom::getHeading() + turret::get_angle();
  while (gamma > 180)
    gamma -= 360;
  while (gamma < -180)
    gamma += 360;
  return gamma;
}

double get_goal_point_distance() {
  arms::Point diff = goal_point - arms::odom::getPosition();
  return arms::length(diff);
}

bool vision_not_working() {
  return (goal_color == 0 && height == 0) || no_new_comm_count > 100;
}

void start_vision() {
  communication->start();
}


void set_vision_offset(int offset) {
  vision_offset = offset;
}

std::tuple<double, double, double> get_turret_pose() {
  // get odom x,y, and heading
  // get turret heading
  arms::Point p = arms::odom::getPosition();
  //-7.5
  // 7.2
  return std::make_tuple(p.x, p.y,
                         arms::odom::getHeading() + turret::get_angle()
                        );
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

  while (true) {
    if (communication->get_read(GOAL_COLOR)) {
      no_new_comm_count = 0;
      goal_color = communication->get_data(GOAL_COLOR);
      left_right = communication->get_data(LEFT_RIGHT);
      height = communication->get_data(HEIGHT);
      width = communication->get_data(WIDTH);
      
      if (goal_color == 1 || goal_color == 2) {
        std::array<double, 2> goal_point_arr = latency_compensator->update_goal_pose(get_goal_distance(), get_goal_gamma());
        goal_point.x = goal_point_arr[0];
        goal_point.y = goal_point_arr[1];
      }
    } else {
      no_new_comm_count += 1;
    }
    printf("No New Com Count: %d\n", no_new_comm_count);
    pros::delay(10);
  }
}
} // namespace vision