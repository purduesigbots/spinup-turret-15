// #include "ARMS/config.h"
#include "comms/comms.hpp"
#include <cstddef>
namespace vision{

    //IRIS COMMUNICATIONS CONSTANTS
    #define START_CHAR 0b11001100
    #define END_CHAR 0b00110011
    #define GOAL_COLOR 0b00000001
    #define LEFT 0b00000010
    #define RIGHT 0b00001000
    #define HEIGHT 0b00000011
    #define WIDTH 0b00000100

    int left;
    int right;
    int height;
    int color;
    int width;

    /**
    *
    * PUBLIC METHODS
    *
    */


   // Make a function that returns the left and right values error
    int get_error(){
      return (left - right);
    }

    void task_func(){
        std::shared_ptr<comms::ReceiveComms> communication = std::make_shared<comms::ReceiveComms>(8, 115200, START_CHAR, END_CHAR); 
        communication->start();
        while(true){
          left = communication->get_data(LEFT);
          right = communication->get_data(RIGHT);
          height = communication->get_data(HEIGHT);
          width = communication->get_data(WIDTH);
          color = communication->get_data(GOAL_COLOR);
          printf("LEFT: %d, RIGHT %d\n", left, right);
          printf("Color: %d\n", color);
          printf("VISION ERROR: %d\n", vision::get_error());
          printf("------------------------\n");
          // printf("\nWidth: %d", width);
          // printf("\nHeight: %d", height);
          pros::delay(10);
        }
        communication->pause();
    }

    void init(){
      pros::Task(task_func, "vision");

    }

} //End namespace vision

