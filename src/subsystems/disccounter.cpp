#include "main.h"
#include "subsystems/subsystems.hpp"
#include "ARMS/config.h"

using namespace pros;

namespace discCounter {

    namespace{ //Anonymous namespace for private data and methods

        /*
        *
        * PRIVATE DATA
        *
        */

        // The line sensor being used for detecting the discs.
        // ADIAnalogIn lineSensor(INTAKE_LINE);

        // The number of discs that the robot currently has. This is made atomic so that
        // we don't get race conditions trying to increment or decrement it.
        std::atomic<int> discCount = 0;

        /**
        * Enumerated class containing the possible states of the disc counter
        * 
        * NO_DISC: The robot is not currently seeing a disc
        *
        * DISC_INTAKE: The robot is currently seeing a disc and is intaking it
        *
        * DISC_OUTTAKE: The robot is currently seeing a disc and is outtaking it
        */
        enum class State {
            NO_DISC,        
            DISC_INTAKE,
            DISC_OUTTAKE
        };

        //Init disc counter to no disc
        State state = State::NO_DISC;

        /**
        *
        * PRIVATE METHODS
        *
        */

        /**
        * Whether the line sensor is currently seeing a disc. 
        *
        * This could probably be drastically improved with a gausian filter 
        * tuned to each competition location, but that's more work. 
        *
        * @return True if the line sensor is currently seeing a disc
        */
        bool seeing_disc() {
            // return lineSensor.get_value() < 2400;
            return false;
        }   
        
        /**
        * The task callback function used to keep track of the number of discs in 
        * the robot. This loops to keep track of the number of discs in the robot. 
        */
        void task_function(void* data) {
            while(true) {
                bool seeingDisc = seeing_disc();
                
                // If we have not yet seen a disc and we just started seeing one,
                // set the appropriate state depending on the intake's direction
                if(state == State::NO_DISC && seeingDisc) {
                    if(intake::intaking()) {
                        state = State::DISC_INTAKE;
                    }
                    else if(intake::outtaking()) {
                        state = State::DISC_OUTTAKE;
                    }
                }

                // If we have been seeing a disc and stopped seeing it, we only
                // increment the disc count if the intake was intaking when we first saw
                // the disc and when we stopped seeing it. Otherwise, the intake's
                // direction must have switched and we was sent back out of the intake
                // and does not affect the disc count.
                else if(
                    state == State::DISC_INTAKE && intake::intaking() // Same direction
                    && !seeingDisc                                    // Stopped seeing
                ) {
                    state = State::NO_DISC;
                    discCount++;
                }
                // Similarly, if we saw a disc when the intake started outtaking, and we
                // stop seeing the disc while it is still outtaking, it must have been
                // ejected from the robot. 
                else if(
                    state == State::DISC_OUTTAKE && intake::outtaking() // Same direction
                    && !seeingDisc                                      // Stopped Seeing
                ) {
                    state = State::NO_DISC;
                    discCount--;
                    printf("ERROR: Disc count somehow negative!!!!\n");
                }

                pros::delay(10);
            }
        }
    }

    /**
    *
    * PUBLIC METHODS (see header for documentation)
    *
    */
    
    void decrement() {
        discCount--;

        if(discCount < 0) {
            printf("ERROR: Disc count somehow negative!!!!\n");
        }
    }

    void initialize() {
        printf("Initializing Disc Counter Subsystem: ");
        pros::Task task(task_function, nullptr, "Disc Counter Task");
        printf("Done\n");
    }

    int disc_count() {
        return discCount;
    }

    int expect(int numDiscs, int timeout) {
        int startTime = pros::millis();
        int startNumDiscs = disc_count();

        while(disc_count() - startNumDiscs < numDiscs) {
            // Check the timeout, and if it's reached, return
            if(pros::millis() - startTime >= timeout && timeout > 0) {
                pros::delay(500);
                return disc_count() - startNumDiscs;
            }

            pros::delay(10);
        }

        pros::delay(500);
        return discCount - startNumDiscs;
    }

    void setNum(int numDiscs){
        discCount = numDiscs;
    }
    
    void debug_screen() {
        lcd2::pages::print_line(4, 0, "Disc Counter Info:");
        lcd2::pages::print_line(4, 1, " Disc Count: %d", disc_count());
        lcd2::pages::print_line(4, 2, " Seeing Disc: %d", seeing_disc());
        //lcd2::pages::print_line(4, 3, " Sensor Value: %d", lineSensor.get_value());
        
        const char* state_str = "";
        switch(state)
        {
        case State::NO_DISC:
            state_str = "NO_DISC";
            break;
        case State::DISC_INTAKE:
            state_str = "DISC_INTAKE";
            break;
        case State::DISC_OUTTAKE:
            state_str = "DISC_OUTTAKE";
            break;
        }
        lcd2::pages::print_line(4, 4, "State: %s", state_str);
    }
}