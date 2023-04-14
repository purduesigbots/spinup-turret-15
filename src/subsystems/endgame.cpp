#include "robot.h"
#include "main.h"
#include "ARMS/config.h"
#include "pros/adi.hpp"

using namespace pros;

namespace endgame {

    namespace{ //Anonymous namespace for private data and methods
        
        /**
        *
        * PRIVATE DATA
        *
        */ 
        
        //State variable: turns true once the endgame is deployed
        bool is_deployed = false;  
        #if !USING_BEN_PNEUMATICS
            pros::ADIDigitalOut leftEndgame (LEFT_ENDGAME);
            pros::ADIDigitalOut rightEndgame (RIGHT_ENDGAME);
        #endif
    }
    
    /**
    *
    * PUBLIC METHODS (see header file for documentation)
    *
    */

    void deploy() {
        is_deployed = true;
        #if USING_BEN_PNEUMATICS
            pneumatics::set_left_endgame(true);
            pneumatics::set_right_endgame(true);
        #else
            leftEndgame.set_value(true);
            rightEndgame.set_value(true);
        #endif
        std::cout << "Endgame launched" << std::endl;
    }

    void deploy_left() {
        #if USING_BEN_PNEUMATICS
            pneumatics::set_left_endgame(true);
        #else 
            leftEndgame.set_value(true);
        #endif
    }

    void deploy_right() {
        #if USING_BEN_PNEUMATICS
            pneumatics::set_right_endgame(true);
        #else
            rightEndgame.set_value(true);
        #endif
    }
}