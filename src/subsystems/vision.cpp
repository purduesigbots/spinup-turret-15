#include "ARMS/config.h"
#include "ARMS/odom.h"
#include "ARMS/point.h"
#include "subsystems/subsystems.hpp"
#include "comms/comms.hpp"
#include "turret.hpp"
#include <cstddef>

/**
* The vision subsystem is responsible for tracking the goal and providing data
* to the turret subsystem.
*
* Template Dependencies: ARMS, comms, serialvision
*
* Subsystem Dependencies: None
*/
namespace vision{

  namespace{ //Anonymous namespace for private methods and data

    /**
    *
    * PRIVATE DATA
    *
    */

    //IRIS COMMUNICATIONS CONSTANTS
    #define START_CHAR 0b11001100
    #define END_CHAR 0b00110011
    #define GOAL_COLOR 0b00000001
    #define LEFT 0b00000010
    #define RIGHT 0b00001000
    #define HEIGHT 0b00000011
    #define WIDTH 0b00000100

    //Various local variables
    int left;
    int right;
    int height;
    int color;
    int previous_color;
    int width;
    int distance = -1;
    double turret_error = 0;

    //Image constants
    const double IMAGE_DIM = 416;
    const double GOAL_HEIGHT_FULL = 13.87;
    const double DISTANCE_SWITCH_THRESHOLD = 40; //ESTIMATE
    const double GOAL_HEIGHT_HALF = 4; //ESTIMATE
    const double GOAL_WIDTH = 15.73;
    const double FOCAL_LENGTH = 0.5;

    // Important, this is the size of the physical sensor, not its height off the
    // ground!
    // https://stackoverflow.com/questions/50125574/calculate-image-size-of-an-object-from-real-size
    const double SENSOR_HEIGHT = 0.3074803;

    //y distance between turret center of rotation and robot center of rotation (negative means turret COR is behind robot COR)
    const double Y_OFFSET_TURET_ROT = -2; 
    const double TURRET_CAMERA_RADIUS = 4; //radius of turret camera from turret COR

    //Target goal
    Goal targColor = Goal::BOTH; //default to both goal colors

    //Estimated goal location
    arms::Point goal_location = {0, -1000}; //Set to 0, -1000 to indicate no goal seen yet

    //Queue of odom states
    std::queue<arms::Point> position_queue;
    //Queue of turret angles
    std::queue<double> angle_queue;

    //Debug flag, counter
    #define VISION_DEBUG false
    int printCounter = 0;

    /**
    *
    * PRIVATE METHODS
    *
    */

    /**
    * Checks if all points in the queue are within a certain distance of each other (indicating a stopped robot)
    *
    * @preturn true if the robot is settled, false if not
    */
    bool robot_is_settled(){
      //Check if the points in the position queue are within a certain distance of each other
      //If they are, the robot is likely settled
      std::queue<arms::Point> temp = position_queue;
      for(int i = 0; i < position_queue.size() - 2; i++){
        arms::Point point1 = temp.front();
        temp.pop();
        arms::Point point2 = temp.front();
        //Use distance formula to calculate distance between point 1 and point 2
        double distance = sqrt(pow(point1.x - point2.x, 2) + pow(point1.y - point2.y, 2));
        if(distance > .5){
          return false; //Return false if any of the points is farther than a half inch from another
        }
      }
      return true; //Return true if all points are within .5" of one another
    }

    /**
    * Checks if the given color is a valid target
    *
    * @param color The color to check
    * @return true if the color is a valid target, false if not
    */
    bool is_valid_target(int color){
        if(targColor == Goal::BOTH){
          return color == 1 || color == 2;
        } else if(targColor == Goal::RED){
          return color == 1;
        } else if(targColor == Goal::BLUE){
          return color == 2;
        } else{
          return false;
        }
    }

    /**
    * Returns the distance to the goal based on image calculations of currently 
    * seen goal
    *
    * @return The distance to the goal in inches
    */
    double get_camera_distance(){
      if(left == 0 || right == 0){
        //If the bounding box is against the frame, we are unlikely to be aiming at the CENTER
        //of the goal, so we return -1 to indicate failure to calculate.
        //NOTE: This eliminates an edge case for half goal detection (YAY!)
        return -1;
      } else if(width / height > 2 && is_valid_target(color)){
        //If the width to height ratio is greater than 2, we are likely seeing half a goal.
        //NO edge cases for thinking it's half a goal when it's not.
        //So, if we think it's half a goal due to w/h ratio and it's a valid target,
        //then we operate under the assumption that it's half a goal and we calculate distance
        //as such:
        return (FOCAL_LENGTH * GOAL_HEIGHT_HALF * IMAGE_DIM) / (height * SENSOR_HEIGHT);
      } else if(is_valid_target(color)){
        //We are seeing a valid target. It is not half a goal. It is not against the frame.
        //There are no remaining edge cases. We calculate distance as normal.
        return (FOCAL_LENGTH * GOAL_HEIGHT_FULL * IMAGE_DIM) / (height * SENSOR_HEIGHT);
      } else{
        //If we are not seeing a valid target, we return -1 to indicate failure to calculate
        return -1;
      }
    }

    /**
    * Updates the goal location based on the current image data
    */
    void update_goal_position(){
      //Calculate pixel to inch ratio for this frame
      double pixel_to_inch = width / GOAL_WIDTH;

      //Calculate the inch error
      double inch_error = pixel_to_inch * .5 * IMAGE_DIM - (left + 0.5 * width);

      //Calculate turret angle error (theta)
      turret_error = atan(inch_error / distance); //radians

      //Calculate camera x and y
      double corr_bot_y = position_queue.front().y;
      double corr_bot_x = position_queue.front().x;
      double corr_turret_angle = angle_queue.front();
      double cam_y = corr_bot_y 
        + sin(arms::odom::getHeading(true)) * Y_OFFSET_TURET_ROT 
        + TURRET_CAMERA_RADIUS * sin(arms::odom::getHeading(true) - corr_turret_angle);
      double cam_x = corr_bot_x 
        + TURRET_CAMERA_RADIUS * cos(arms::odom::getHeading(true) - corr_turret_angle);

      //Calculate goal x and y
      double goal_y = cam_y + distance * sin(arms::odom::getHeading(true) - corr_turret_angle - turret_error);
      double goal_x = cam_x + distance * cos(arms::odom::getHeading(true) - corr_turret_angle - turret_error);

      //Update goal location
      goal_location = {goal_x, goal_y};
    }

    /**
    * Asynchronous task function for vision subsystem
    */
    void task_func(){

      //Create communications object
      std::shared_ptr<comms::ReceiveComms> communication = std::make_shared<comms::ReceiveComms>(
        IRIS_PORT,  //Port
        115200,     //Baud rate
        START_CHAR, //Start character
        END_CHAR);  //End character

      //Start communications
      communication->start();

      //Main loop
      while(true){
        //Put in new odom state
        position_queue.push(arms::odom::getPosition());
        //Put in new turet angle
        angle_queue.push(turret::get_angle());
        //if there are more than 4 states in the position queue, remove the oldest one
        if(position_queue.size() > 4){
          position_queue.pop();
        }
        //if there are more than 4 states in the angle queue, remove the oldest one
        if(angle_queue.size() > 4){
          angle_queue.pop();
        }

        //Get data from IRIS
        left = communication->get_data(LEFT);
        right = communication->get_data(RIGHT);
        height = communication->get_data(HEIGHT);
        width = communication->get_data(WIDTH);
        previous_color = color;
        color = communication->get_data(GOAL_COLOR);

        //Debug statements
        if(VISION_DEBUG && printCounter % 5 == 0){
          printf("LEFT: %d, RIGHT %d\n", left, right);
          printf("Color: %d\n", color);
          printf("VISION ERROR: %d\n", get_error());
          printf("------------------------\n");
        }
        printCounter++;

        //Loop delay
        pros::delay(10);
      }
    }
  } //End anonymous namespace
    
    /**
    *
    * PUBLIC METHODS (see header file for documentation)
    *
    */
    
    int get_error(){
      if(robot_is_settled()){
        //If we are not moving we do not have to lead our shot. Return turret's theta error in degrees
        return turret_error * 180 / M_PI;
      } else{
        //If we are moving, we have to lead our shot. 
        //Calculate the x and y velocities of the robot
        arms::Point curPos = position_queue.front();
        std::queue<arms::Point> temp = position_queue;
        temp.pop();
        arms::Point prevPos = temp.front();
        //calculate x and y velocity assuming no acceleration (teehee)
        double x_vel = (curPos.x - prevPos.x) / 0.01;
        double y_vel = (curPos.y - prevPos.y) / 0.01;
        //calculate the magnitude of the velocity perpendicular to the goal
        double perp_vel = fabs(x_vel * sin(turret_error) + y_vel * cos(turret_error)); //CHECK THIS
      }
    }

    void init(){
      pros::Task(task_func, "vision");
    }

    bool is_working(){
      //Color = 1 if red, 2 if blue, 3 if no goal but recieving data, 0 if no data
      return color != 0; 
    }

    double get_distance(){
      //Attempt to calculate camera distance
      double cameraDistance = get_camera_distance();
      if(cameraDistance != -1){
        //If we successfully calculated the camera distance, perform
        //an update of the goal location and return the distance
        distance = cameraDistance;
        update_goal_position();

        //Return distance
        return distance;
      } else if (goal_location.y != -1000){
        //Calculate based on saved goal location, including offset for camera

        //Calculate camera x and y
        double cam_y = arms::odom::getPosition().y 
          + sin(arms::odom::getHeading(true)) * Y_OFFSET_TURET_ROT 
          + TURRET_CAMERA_RADIUS * sin(arms::odom::getHeading(true) - turret::get_angle());
        double cam_x = arms::odom::getPosition().x 
          + TURRET_CAMERA_RADIUS * cos(arms::odom::getHeading(true) - turret::get_angle());
        
        //Calculate distance
        distance = sqrt(pow(goal_location.x - cam_x, 2) 
          + pow(goal_location.y - cam_y, 2));

        //Return distance
        return distance;
      }
      return -1; //Return -1 if no goal saved and/or none in sight
    }

} //End namespace vision
