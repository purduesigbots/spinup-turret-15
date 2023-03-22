#include "main.h"
#include "ARMS/config.h"
#include "subsystems/subsystems.hpp"

namespace deflector {

    namespace{ //Anonymous namespace for private data and methods
        
        /*
        *
        * PRIVATE DATA
        *
        */

        //Piston declaration
        pros::ADIDigitalOut deflector_piston(DEFLECTOR_PISTON);
        
        //Whether or not the deflector is up
        bool isUp = true;
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
        deflector_piston.set_value(!isUp); //state variable is reversed in relation to piston, hence the not operator
    }

    bool is_up() {
        return isUp;
    }

} // namespace deflector
