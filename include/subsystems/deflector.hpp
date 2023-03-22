#pragma once

/**
* The deflector subsystem is responsible for moving the deflector up and down in order to 
* change the trajectory of discs as they leave the flywheel.
*/
namespace deflector {

    /**
     * Puts the deflector in the up position
     */
    void up();

    /**
     * Puts the deflector in the down position
     */
    void down();

    /**
     * Toggle the deflector's position 
     */
    void toggle();

    /**
     * Returns whether the deflector is up or not
     * 
     * @return True if the deflector is the up position 
     */
    bool is_up();

}