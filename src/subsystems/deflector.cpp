#include "ARMS/config_silver.h"
#include "main.h"
#include "ARMS/config.h"
#include "pros/adi.hpp"
#include "subsystems/subsystems.hpp"

namespace deflector {

    namespace{ //Anonymous namespace for private data and methods
        
        /*
        *
        * PRIVATE DATA
        *
        */
        
        //Whether or not the deflector is up
        bool isUp = true;

        pros::ADIDigitalOut deflectorPiston (DEFLECTOR_PISTON);
    }
    
    /**
    *
    * PUBLIC METHODS (all comment)
    *
    */

    void up() {
        if(!isUp) {
            toggle(); //toggle if not already up
        }
    }

    void down() {
        if(isUp) {
            toggle(); //toggle if not already down
        }
    }

    void toggle(){
        isUp = !isUp; //flip state variable
        //Activate solenoid
        deflectorPiston.set_value(!isUp); //state variable is reversed in relation to piston, hence the not operator
    }

    bool is_up() {
        return isUp;
    }

} // namespace deflector
