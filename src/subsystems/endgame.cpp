#include "subsystems/endgame.hpp"
#include "main.h"
#include "ARMS/config.h"

using namespace pros;

namespace endgame {

    namespace{ //Anonymous namespace for private data and methods
        
        /**
        *
        * PRIVATE DATA
        *
        */ 
        
        //The piston used to deploy the endgame
        ADIDigitalOut piston(ENDGAME_PISTON);
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
        piston.set_value(true);
        std::cout << "Endgame launched" << std::endl;
    }
}