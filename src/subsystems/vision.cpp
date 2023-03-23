#include "ARMS/config_silver.h"
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

    uint64_t left;
    uint64_t right;
    uint64_t height;
    uint64_t color;
    uint64_t width;

    /**
    *
    * PUBLIC METHODS
    *
    */

    void task_func(void* param){
        std::shared_ptr<comms::ReceiveComms> communication = std::make_shared<comms::ReceiveComms>(IRIS_PORT, 115200, START_CHAR, END_CHAR); 
        communication->start();
        while(true){
          left = communication->get_data(LEFT);
          right = communication->get_data(RIGHT);
          height = communication->get_data(HEIGHT);
          width = communication->get_data(WIDTH);
          color = communication->get_data(GOAL_COLOR);
          printf("\nLEFT: %llu, RIGHT %llu", left, right);
          printf("\nColor: %llu", color);
          printf("\nWidth: %llu", width);
          printf("\nHeight: %llu", height);
          pros::delay(10);
        }
    }

    void init(){
      pros::Task(task_func, "vision");
    }

} //End namespace vision

