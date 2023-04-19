#include "ARMS/config.h"
#include "ARMS/odom.h"
#include "ARMS/point.h"
#include "flywheel.hpp"
#include "subsystems/subsystems.hpp"
#include "comms/comms.hpp"
#include "turret.hpp"
#include "vision.hpp"
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

    #define SMA_LEN 4

    //Various local variables
    int left;
    int right;
    int height;
    int color;
    int previous_color;
    int width;
    double distance = -1;
    double turret_error = 0;
    double inch_error = 0;
    int odom_settled_time = 0;

    //Image constants
    const double IMAGE_DIM = 416;
    const double GOAL_HEIGHT_FULL = 13.87;
    const double DISTANCE_SWITCH_THRESHOLD = 40; //ESTIMATE
    const double GOAL_HEIGHT_HALF = 10; //ESTIMATE
    const double GOAL_WIDTH_HALF = 12; //ESTIMATE
    const double GOAL_WIDTH = 15.73;
    const double GOAL_RADIUS = 8; //So that we calculate the center of the goal not the edge of it
    const double FOCAL_LENGTH = 0.5;
    const double FOV = 81 * M_PI / 180; //81 degrees in radians

    // Important, this is the size of the physical sensor, not its height off the
    // ground!
    // https://stackoverflow.com/questions/50125574/calculate-image-size-of-an-object-from-real-size
    const double SENSOR_HEIGHT = 0.3074803;

    //y distance between turret center of rotation and robot center of rotation (negative means turret COR is behind robot COR)
    const double Y_OFFSET_TURET_ROT = -1; 
    const double TURRET_CAMERA_RADIUS = 4.75; //radius of turret camera from turret COR

    //Target goal
    Goal target_color = Goal::BOTH; //default to both goal colors

    //Estimated goal location
    arms::Point goal_location = {0, -1000}; //Set to 0, -1000 to indicate no goal seen yet
    arms::Point blue_goal_location = {0, -1000};
    arms::Point red_goal_location = {0, -1000};
    std::deque<arms::Point> sma;

    //Queue of odom states
    std::queue<arms::Point> position_queue;
    //Queue of turret angles
    std::queue<double> angle_queue;
    //Queue of robot headings
    std::queue<double> heading_queue;

    //Debug flag, counter
    #define VISION_DEBUG false
    int printCounter = 0;

    //Shoot while moving flag
    #define SHOOT_WHILE_MOVING false

    //Odom guessing flag
    #define ODOM_GUESS_FLAG true

    /**
    *
    * PRIVATE METHODS
    *
    */

    /**
    * Calculate the angular coordinates of a pixel location in an image 
    * based on the field of view of the camera
    *
    * @param pixelCoord The pixel coordinate to calculate the angle for
    * @param imageDim The dimension of the image in pixels
    * @param cameraFov The field of view of the camera in degrees
    * @return The angle of the pixel location in radians
    */
    float pixelToAngle(float pixelCoord, int imageDim, float cameraFov) {
        // Calculate the horizontal angle of view for the camera based on its field of view
        float hFov = cameraFov * (M_PI / 180.0); // Convert degrees to radians

        // Calculate the angle of the pixel location relative to the center of the image in the horizontal direction
        float xAngle = (pixelCoord - imageDim / 2.0) * hFov / imageDim;

        // Return the resulting angle in radians
        return xAngle;
    }

    /**
    * Constrains an inputted angle (radians) to the -pi --> pi regime
    *
    * @return An equivalent angle (radians) where -pi <= angle < pi
    */
    double constrainAngle(double angle){
        while(angle >= M_PI){
          angle -= 2 * M_PI;
        }
        while(angle < -M_PI){
          angle += 2 * M_PI;
        }
        return angle;
    }

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
        double point_distance = sqrt(pow(point1.x - point2.x, 2) + pow(point1.y - point2.y, 2));
        if(point_distance > .001){
          return false; //Return false if any of the points is farther than a half inch from another
        }

        //Check omega
        double omega = fabs(angle_queue.front() - angle_queue.back());
        if(omega > 0.001){ //Threshold is in radians per second 
          return false;
        }
      }
      return true; //Return true if all points are within .5" of one another and not really turing much
    }

    /**
    * Checks if the given color is a valid target
    *
    * @param color The color to check
    * @return true if the color is a valid target, false if not
    */
    bool is_valid_target(int color){
        if(!is_working()) return false;
        if(target_color == Goal::BOTH){
          return color == 1 || color == 0;
        } else if(target_color == Goal::RED){
          return color == 1;
        } else if(target_color == Goal::BLUE){
          return color == 0;
        }
        return false;
    }

    /**
    * Returns the distance to the goal based on image calculations of currently 
    * seen goal
    *
    * @return The distance to the goal in inches
    */
    double get_camera_distance(){
      double dist;
      if(left <= 5 || right <= 5){
        //If the bounding box is against the frame, we are unlikely to be aiming at the CENTER
        //of the goal, so we return -1 to indicate failure to calculate.
        //NOTE: This eliminates an edge case for half goal detection (YAY!)
        dist = -1;
        if(is_working() && color != 3){
          //If we are seeing a goal at all, return -2 to indicate that we are seeing a partial goal against the frame
          dist = -2;
        }
      } else if(width > height && is_valid_target(color)){
        //If the width to height ratio is greater than 2, we are likely seeing half a goal.
        //NO edge cases for thinking it's half a goal when it's not.
        //So, if we think it's half a goal due to w/h ratio and it's a valid target,
        //then we operate under the assumption that it's half a goal and we calculate distance
        //as such:
        dist = -1; //Does not work because it's too close--use odom distance
      } else if(is_valid_target(color)){
        //We are seeing a valid target. It is not half a goal. It is not against the frame.
        //There are no remaining edge cases. We calculate distance as normal.
        dist = GOAL_RADIUS + (FOCAL_LENGTH * GOAL_HEIGHT_FULL * IMAGE_DIM) / (height * SENSOR_HEIGHT);
        dist *= 1.02; //"shit happens" corrective factor - JBH 4/7/23
      }
      else{
        //If we are not seeing a valid target, we return -1 to indicate failure to calculate
        dist = -1;
      }
      //If the distance is greater than 130 inches, we return -1 to indicate failure to calculate
      return dist < 130? dist : -1; 
    }

    /**
    * Updates the goal location based on the current image data
    */
    void update_goal_position_and_turret_error(){
      //Calculate pixel to inch ratio for this frame
      double pixel_to_inch = GOAL_WIDTH / width;

      //Calculate the inch error
      inch_error = pixel_to_inch * (.5 * IMAGE_DIM - (left + 0.5 * width));

      //Calculate turret angle error (theta)
      turret_error = constrainAngle(atan(inch_error / distance)); //radians

      //Only allow update to occur if turret error is less than 10 degrees
      //Prevent update if not settled (want to only do this when we are still and have a good picture of goal)
      if(fabs(turret_error) < 10.0 && robot_is_settled()){ 
        //Calculate camera x and y
        double corr_bot_y = position_queue.front().y;
        double corr_bot_x = position_queue.front().x;
        double corr_heading_rad = heading_queue.front();
        double corr_turret_angle = angle_queue.front();
        double cam_y = corr_bot_y 
          + sin(corr_heading_rad) * Y_OFFSET_TURET_ROT 
          + TURRET_CAMERA_RADIUS * sin(corr_heading_rad - corr_turret_angle);
        double cam_x = corr_bot_x 
          + cos(corr_heading_rad) * Y_OFFSET_TURET_ROT
          + TURRET_CAMERA_RADIUS * cos(corr_heading_rad - corr_turret_angle);
    
        //Calculate goal x and y
        double goal_y = cam_y + distance * sin(corr_heading_rad + corr_turret_angle + turret_error);
        double goal_x = cam_x + distance * cos(corr_heading_rad + corr_turret_angle + turret_error);
        if(VISION_DEBUG && (printCounter + 1) % 5 == 0){
          // printf("Heading: %3.2f, Total %3.2f, Turret: %3.2f\n", corr_heading_rad * 180/M_PI, (corr_heading_rad + corr_turret_angle + turret_error) * 180 / M_PI, corr_turret_angle * 180 / M_PI);
        }
        //Update goal location
        arms::Point current = {goal_x, goal_y};
        arms::Point sum = goal_location * sma.size();
        sma.push_front(current);
        arms::Point last = {0,0};
        if (sma.size() > SMA_LEN) {
          last = sma.back();
          sma.pop_back();
        }
        goal_location = (sum + current - last) / sma.size();
        if(color == 1) red_goal_location = goal_location;
        else if(color == 0) blue_goal_location = goal_location;
      }
    }

    /**
    * Calculates the distance to the goal based on the current image data
    * or the last known goal location
    *
    * @return The distance to the goal FROM THE CAMERA in inches
    */
    double calculate_distance(double cameraDistance){
      //Attempt to calculate camera distance
      if(cameraDistance > 0){
        //If we successfully calculated the camera distance, perform
        //an update of the goal location and return the distance
        update_goal_position_and_turret_error();

        //Return distance
        return cameraDistance;
      } else if (goal_location.y != -1000){
        //Calculate based on saved goal location, including offset for camera
        //Calculate camera x and y
        double cam_y = arms::odom::getPosition().y 
          + sin(arms::odom::getHeading(true)) * Y_OFFSET_TURET_ROT 
          + TURRET_CAMERA_RADIUS * sin(arms::odom::getHeading(true) - turret::get_angle(true));
        double cam_x = arms::odom::getPosition().x 
          + TURRET_CAMERA_RADIUS * cos(arms::odom::getHeading(true) - turret::get_angle(true));
        
        //Calculate distance
        distance = sqrt(pow(goal_location.x - cam_x, 2) 
          + pow(goal_location.y - cam_y, 2));

        //Return distance
        return distance;
      }
      return cameraDistance;
    }

    double calculate_turret_error_odom(double cameraDistance){
      if((cameraDistance < 0 || turret_error == -1000) && ODOM_GUESS_FLAG){
        //If we failed to calculate the camera distance, calculate turret error based on 
        //current robot location, current goal location, and current turret angle.

        //If in skills/practice mode:
        //  1. Check if we valid odom locations for BOTH red and blue goals
        //  2. If so, calculate turret error for both goals
        //  3. Choose the goal with the smallest turret error and set goal location to that goal for final calculation
        //If the first check fails, we will use the last known goal location
        if(target_color == Goal::BOTH && blue_goal_location.y != -1000 && red_goal_location.y != -1000){
          double turret_error_blue = 
            atan2((blue_goal_location.y - arms::odom::getPosition().y) , (blue_goal_location.x - arms::odom::getPosition().x)) 
            - arms::odom::getHeading(true) 
            - turret::get_angle(true);
          double turret_error_red = 
            atan2((red_goal_location.y - arms::odom::getPosition().y) , (red_goal_location.x - arms::odom::getPosition().x)) 
            - arms::odom::getHeading(true) 
            - turret::get_angle(true);
          turret_error_blue = constrainAngle(turret_error_blue);
          turret_error_red = constrainAngle(turret_error_red);
          if(fabs(turret_error_blue) <= fabs(turret_error_red)){
            goal_location = blue_goal_location;
          } else{
            goal_location = red_goal_location;
          }
        } 
        turret_error = 
          atan2((goal_location.y - arms::odom::getPosition().y) , (goal_location.x - arms::odom::getPosition().x)) 
          - arms::odom::getHeading(true) 
          - turret::get_angle(true);
        return constrainAngle(turret_error); //reversed to follow convention
      } else if(cameraDistance == -2){
        //If we are in the edge case where we are seeing a goal against the frame,
        //we calculate the error based on odom distance to goal point and how wide 
        //the goal would be if we could see the whole thing compared to how wide it is
        //in the image. We then use this error as turret_error.
        //We do not update the goal location in this case. That will be accomplished once
        //this edge case kicks the camera's view of the goal fully into frame.
        //Calculate pixel to inch ratio for this frame

        double pixel_to_inch = distance > DISTANCE_SWITCH_THRESHOLD? 
          GOAL_HEIGHT_FULL / width: 
          GOAL_HEIGHT_HALF / width;

        //Approximate simple version of turret error
        turret_error = constrainAngle((M_PI / 180) * (right > 5? -10 : 10));
        return turret_error;
      }
      return turret_error; //NOTE: TURRET ERROR WAS ALREADY CONSTRAINED IF WE REACH THIS RETURN
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
      int no_new_comm_count = 0;

      //Main loop
      while(true){
        //Put in new odom state
        position_queue.push(arms::odom::getPosition());
        //Put in new turet angle
        angle_queue.push(turret::get_angle(true));
        //Put in heading
        heading_queue.push(arms::odom::getHeading(true));
        //if there are more than 4 states in the position queue, remove the oldest one
        if(position_queue.size() > 4){
          position_queue.pop();
        }
        //if there are more than 4 states in the angle queue, remove the oldest one
        if(angle_queue.size() > 4){
          angle_queue.pop();
        }
        //if there are more than 4 states in the heading queue, remove the oldest one
        if(heading_queue.size() > 4){
          heading_queue.pop();
        }

        double cameraDistance = -1;

        if (communication->get_read(WIDTH)) {
          //Get data from IRIS
          no_new_comm_count = 0;
          left = communication->get_data(LEFT);
          right = communication->get_data(RIGHT);
          height = communication->get_data(HEIGHT);
          width = communication->get_data(WIDTH);
          previous_color = color;
          color = communication->get_data(GOAL_COLOR);
          cameraDistance = get_camera_distance();
        } else {
          no_new_comm_count++;
        }

        //Update distance
        turret_error = -1000;
        cameraDistance = get_camera_distance();
        distance = calculate_distance(cameraDistance);
        turret_error = calculate_turret_error_odom(cameraDistance);

        //Debug statements
        if(VISION_DEBUG && printCounter % 5 == 0){
          printf("---------------------------------------------------------\n");
          printf("LEFT: %d, RIGHT %d, WIDTH %d, HEIGHT %d\n", left, right, width, height);
          printf("Color: %1d, Distance: %3f\n", color, distance);
          // printf("Width: %3d, Height: %3d\n", width, height);
          printf("Camera Distance: %f, inch error: %f\n", distance, inch_error);
          printf("Turret Heading: %3.2f\n", turret::get_angle());
          printf("Goal X: %3.2f, Goal Y: %3.2f\n", goal_location.x, goal_location.y);
          printf("VISION ERROR: %f\n", get_error(false));
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
    
  double get_error(bool radians){
    if(SHOOT_WHILE_MOVING && robot_is_settled()){
      //If we are not moving we do not have to lead our shot. Return turret's theta error in degrees
      return radians? turret_error : turret_error * 180 / M_PI;;
    } else if(SHOOT_WHILE_MOVING){
      //If we are moving, we have to lead our shot. 
      //Calculate the x and y velocities of the robot
      std::queue<arms::Point> temp = position_queue;
      temp.pop();
      temp.pop();
      arms::Point prevPos = temp.front();
      temp.pop();
      arms::Point curPos = temp.front();
      //calculate x and y velocity assuming no acceleration (teehee)
      double x_vel = (curPos.x - prevPos.x) / 0.01;
      double y_vel = (curPos.y - prevPos.y) / 0.01;
      double vel_heading = atan2(y_vel, x_vel); //radians, accounts for getting pushed or whatever
      //calculate the magnitude of the velocity
      double vel = sqrt(x_vel * x_vel + y_vel * y_vel);
      //magnitude of velocity perpendicular to a line drawn from the robot to the goal
      double perp_vel = 
        vel * sin(vel_heading - arms::odom::getHeading(true) 
          + turret::get_angle(true) + turret_error);
      //calculate the velocity of the disc as it leaves the flywheel
      double disc_vel = 1.5 * (flywheel::current_speed() * (3600.0 / 200.0) * (360.0 / 60) * (M_PI / 180.0));
      //get angle between the disc velocity and the perpendicular velocity
      turret_error = sin(perp_vel / disc_vel); //radians
      //return the error in degrees
      return radians? turret_error : turret_error * 180 / M_PI;;
    } else{
      //If we are moving and we are not allowed to shoot while moving, return standard error
      return radians? turret_error : turret_error * 180 / M_PI;
    }
    if(!is_working() || color == 3){
      return 0;
    }
  }

  void init(){
    pros::Task(task_func, "vision");
  }

  bool is_working(){
    //Color = 1 if red, 0 if blue, 3 if no goal but recieving data, 0 and all other values 0 if no data
    return !(color == 0 && width == 0 && height == 0);
  }

  double get_distance(){
    return distance;
  }
  
  void set_targ_goal(Goal targ){
    //Set target color
    target_color = targ;
  }

  Goal get_targ_goal(){
    //Get target color
    return target_color;
  }

  arms::Point get_goal_pos(){
    return goal_location;
  }

} //End namespace vision
