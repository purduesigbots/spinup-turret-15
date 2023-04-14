#include "ARMS/config_silver.h"
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

        pros::ADIDigitalOut leftEndgame (LEFT_ENDGAME);
        pros::ADIDigitalOut rightEndgame (RIGHT_ENDGAME);
    }
    
    /**
    *
    * PUBLIC METHODS (see header file for documentation)
    *
    */

    void deploy() {
        is_deployed = true;
        leftEndgame.set_value(true);
        rightEndgame.set_value(true);
        std::cout << "Endgame launched" << std::endl;
    }

    void deploy_left() {
        leftEndgame.set_value(true);
    }

    void deploy_right() {
        rightEndgame.set_value(true);
    }
}