// clang-format off
#include "vision.h"
#include "subsystems.h"
#include "comms/comms.hpp"
#include "ARMS/odom.h"
#include "pros/misc.h"
#include "LatPullDown/Oak_1_latency_compensator.hpp"
// clang-format on

#define START_CHAR 0b11001100
#define END_CHAR 0b00110011

#define GOAL_COLOR 0b00000001
#define LEFT_RIGHT 0b00000010
#define HEIGHT 0b00000011

const double IMAGE_HEIGHT = 416;
const double GOAL_HEIGHT = 13.87;
const double FOCAL_LENGTH = 0.1893701; 

// Important, this is the size of the physical sensor, not its height off the
// ground! 
// https://stackoverflow.com/questions/50125574/calculate-image-size-of-an-object-from-real-size
const double SENSOR_HEIGHT = 0.3074803;

// std::tuple<double,double,double> get_turret_pose() {
// 	// get odom x,y, and heading
// 	// get turret heading
// 	arms::Point p =  arms::odom::getPosition();
// 	//-7.5
// 	//7.2
// 	return std::make_tuple(p.x, p.y, arms::odom::getHeading() -
// turret::get_position());
// }

// std::tuple<double,double> get_goal_vector(std::tuple<double,double,double>
// pose) {
// 	// get goal distance
// 	// calculate vector to the goal
// 	return std::make_tuple(0,0);
// }

// double get_latency() {
// 	// get latency from the ReceiveComms class
// 	return 35;
// }

// Oak_1_latency_compensator latency_compensator(
// 	5, // max buffer of 7
// 	10, // get odom position every 10ms
// 	get_turret_pose, // function to get pose of the turret
// 	get_goal_vector, // function to calulate the vector to the goal based on
// the distance 	get_latency // function to get the latency of the
// current frame
// 	);

namespace vision {

std::shared_ptr<comms::ReceiveComms> communication;

void init() {
  communication =
      std::make_shared<comms::ReceiveComms>(8, 115200, START_CHAR, END_CHAR);
}

double get_goal_distance()
{
  return (FOCAL_LENGTH * GOAL_HEIGHT * IMAGE_HEIGHT) / 
         (communication->get_data(HEIGHT) * SENSOR_HEIGHT);
}

std::tuple<double, double, double> get_turret_pose() {
  // get odom x,y, and heading
  // get turret heading
  arms::Point p = arms::odom::getPosition();
  //-7.5
  //7.2
  return std::make_tuple(p.x, p.y, arms::odom::getHeading() - turret::get_position());
}

double get_latency() {
  return 40; // 40ms latency
}

Oak_1_latency_compensator latency_compensator(
    7, // max buffer of 7
    10, // get odom position every 10ms
    get_turret_pose, // function to get pose of the turret
    get_goal_distance, // function to calulate the vector to the goal based on the distance
    get_latency // function to get the latency of the current frame
);

void task() {
  communication->start();
  float previous_speed = 0;

  float previous_height;
  float previous_lr;
  float previous_color;
  float counter = 0;

  while (true) {
    uint64_t color = communication->get_data(GOAL_COLOR);
    uint64_t lr = communication->get_data(LEFT_RIGHT);
    uint64_t height = communication->get_data(HEIGHT);

    float speed = (140 - (float)lr) / 140 * 100;

    // float min_speed = 10.0;
    // float deadzone = 3.0;

    // if (speed < 0.0) {
    //   if (speed > -min_speed) {
    //     speed = -min_speed;
    //   }
    // } else if (speed > 0.0) {
    //   if (speed < min_speed) {
    //     speed = min_speed;
    //   }
    // } else {
    //   speed = 0.0;
    // }

    // if (lr - 140 <= deadzone) {
    //   speed = 0.0;
    // }

    // 0 is red
    // 1 is blue
    // 3 is nothing detected

    double turn_degrees = latency_compensator.get_new_goal_distance();

    if (color == 0) {
      speed = 0;
    }

    printf("Color:      %llu\n", color);
    printf("Left/Right: %llu\n", lr);
    printf("Height:     %llu\n", height);
    printf("Speed:      %f\n", turret::speed);
    printf("Time:       %f\n", counter);

    if (previous_height != height || previous_lr != lr ||
        previous_speed != speed || previous_color != color) {
      counter = 0;
    } else {
      counter += 10;
    }

    previous_height = height;
    previous_lr = lr;
    previous_speed = speed;
    previous_color = color;
    printf("----------------\n");

    turret::move(turn_degrees);
    
    pros::delay(10);
  }
}
} // namespace vision