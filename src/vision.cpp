#include "vision.h"
#include "subsystems/subsystems.hpp"
#include "comms/comms.hpp"
#include "ARMS/odom.h"
#include "pros/misc.h"
#include "LatPullDown/Oak_1_latency_compensator.hpp"
#include "ARMS/config.h"

//IRIS COMMUNICATIONS MACROS
#define START_CHAR 0b11001100
#define END_CHAR 0b00110011
#define GOAL_COLOR 0b00000001
#define LEFT_RIGHT 0b00000010
#define HEIGHT 0b00000011
#define WIDTH 0b00000100

namespace vision{
  
  namespace{ //Anonymous namespace for private data and methods

    /*
    *
    * PRIVATE DATA
    *
    */

    const std::tuple<double, double> GOAL_POS = {50 * sqrt(2), 0};

    const double IMAGE_HEIGHT = 416;
    const double GOAL_HEIGHT = 13.87;
    const double GOAL_WIDTH = 16;
    const double FOCAL_LENGTH = 0.5;

    uint64_t color = 0;

    // Important, this is the size of the physical sensor, not its height off the
    // ground!
    // https://stackoverflow.com/questions/50125574/calculate-image-size-of-an-object-from-real-size
    const double SENSOR_HEIGHT = 0.3074803;

    std::shared_ptr<comms::ReceiveComms> communication;

    double vision_offset = 240;

    //NOT BEING USED - Jack
    // Oak_1_latency_compensator *latency_compensator;

    /**
    *
    * PRIVATE METHODS
    *
    */

    /**
     * Gets the position of the robot except heading is the turret's global heading instead of the chassis'
     *
     * @return an arms pose comprised of the robot's x, y and the turret's global angle
     */
    std::tuple<double, double, double> get_turret_pose() {
      arms::Point p = arms::odom::getPosition();
      return std::make_tuple(p.x, p.y, arms::odom::getHeading() + turret::get_angle() +get_goal_gamma());
    }

    /**
     * gets the latency of the iris system
     *
     * @return the latency of the iris system in ms
     */
    double get_latency(){
      return 40; //40ms latency (for lat comp thingy)
    }

    /**
     * Task function governing communication in between project IRIS and the rest of the program
     */
    void task_func() {
      communication->start(); //start comms with iris box

      //THIS IS NOT BEING USED, LEAVING IT HERE FOR NOW UNTIL LAT COMP IS DONE - Jack
      /* latency_compensator = new Oak_1_latency_compensator( //init latency compensator
        7,                 // max buffer of 7
        10,                // get odom position every 10ms
        get_turret_pose,   // function to get pose of the turret
        get_goal_distance, // function to calulate the vector to the goal based on our distance from it
        get_latency,       // 40 ms latency
        false);
      */

      //THESE ARE NOT BEING USED, leaving here in case that changs - Jack
      // float previous_height;
      // float previous_lr;
      // float previous_color;

      //MAIN TASK LOOP
      while (true) {
        color = communication->get_data(GOAL_COLOR);
        uint64_t lr = communication->get_data(LEFT_RIGHT);
        uint64_t height = communication->get_data(HEIGHT);
        uint64_t width = communication->get_data(WIDTH);

        //NOT BEING USED - Jack
        // previous_height = height; 
        //NOT BEING USED - Jack
        // previous_lr = lr;
        //NOT BEING USED - Jack
        // previous_color = color;

        //Loop delay
        pros::delay(10);
      }
    }
  }

  /**
  *
  * PUBLIC METHODS (commented in the header file)
  *
  */

  void init() {
    communication = std::make_shared<comms::ReceiveComms>(IRIS_PORT, 115200, START_CHAR, END_CHAR);
    pros::Task vision_task(task_func, "Vision task");
  }

  double get_goal_gamma() {
    return atan2((GOAL_WIDTH / (double)communication->get_data(WIDTH) *
                  (vision_offset - (double)communication->get_data(LEFT_RIGHT))),
                get_goal_distance()) *
          (180 * M_1_PI);
  }

  double get_goal_distance() {
    return (FOCAL_LENGTH * GOAL_HEIGHT * IMAGE_HEIGHT) /
          (communication->get_data(HEIGHT) * SENSOR_HEIGHT);
  }

  bool vision_not_working() {
    return communication->get_data(GOAL_COLOR) == 0 && communication->get_data(HEIGHT) == 0;
  }

  void set_vision_offset(int offset) {
    vision_offset = offset;
  }

  int get_goal_color() {
    return color; // Last seen color (at least it is 40ms old)
  }

  bool get_goal_detected(){
    return color != 3; //if 3, then no goal detected
  }
}