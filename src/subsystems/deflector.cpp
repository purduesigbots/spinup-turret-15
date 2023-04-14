#include "robot.h"
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

        #if !USING_BEN_PNEUMATICS
            pros::ADIDigitalOut deflectorPiston (DEFLECTOR);
        #endif
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
        #if USING_BEN_PNEUMATICS
            pneumatics::set_deflector(!isUp);
        #else
            deflectorPiston.set_value(!isUp); //state variable is reversed in relation to piston, hence the not operator
        #endif
    }

    bool is_up() {
        return isUp;
    }

} // namespace deflector
