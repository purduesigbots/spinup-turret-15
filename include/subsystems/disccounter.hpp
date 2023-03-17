#pragma once

/**
 * The disc counting subsystem is in charge of keeping track of when discs enter
 * or leave the robot. 
 */
namespace disccounter {

    /**
    * Initializes the disc counter subsystem: Creates tasks, sets defaults, etc. 
    */
    void initialize();

    /**
    * The number of discs currently in the robot 
    *
    * @return The number of discs currently in the robot
    */
    int disc_count();

    /**
    * Tells the halt until a certain number of discs are taken up.
    * 
    * This is intended for auton programming. This allows the program to wait
    * until a number of discs are picked up, but give up 
    * 
    * @param numDiscs How many discs to expect/pick up
    * @param timeout  How long to try before giving up
    * 
    * @return The number of discs that were detected when picking up
    */
    int expect(int numDiscs, int timeout = 5000);

    /**
    * Tells the system how many discs are inside it (i.e. preloads)
    * 
    * This is for overriding the current count
    * 
    * @param numDiscs How many discs are in the system
    */
    void setNum(int numDiscs);

    /**
    * Renders the debug screen to the LLEMU display 
    */
    void debug_screen();

}