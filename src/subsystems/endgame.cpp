#include "main.h"
#include "ARMS/config.h"
#include "subsystems/pneumatics.hpp"

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
    }
    
    /**
    *
    * PUBLIC METHODS (see header file for documentation)
    *
    */

    void deploy() {
        is_deployed = true;
        pneumatics::set_left_endgame(true);
        pneumatics::set_right_endgame(true);
        std::cout << "Endgame launched" << std::endl;
    }

    void deploy_left() {
        pneumatics::set_left_endgame(true);
    }

    void deploy_right() {
        pneumatics::set_right_endgame(true);
    }
}