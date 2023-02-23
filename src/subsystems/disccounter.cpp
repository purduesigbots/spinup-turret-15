#include "subsystems/disccounter.hpp"

#include "main.h"
#include "subsystems/subsystems.hpp"

#include <atomic>
#include <memory>

using namespace pros;

namespace disccounter {

// The line sensor being used for detecting the discs.
ADIAnalogIn lineSensor('f');

// The number of discs that the robot currently has. This is made atomic so that
// we don't get race conditions trying to increment or decrement it.
std::atomic<int> discCount = 0;

// List of states by this subsystem. 
enum class State {
    NO_DISC,        // We have not started seeing a disc
    DISC_INTAKE,    // We started seeing a disc while the intake was intaking
    DISC_OUTTAKE    // We started seeing a disc while the intake was outtaking
};
State state = State::NO_DISC;

// Returns whether the line sensor is currently seeing a disc. This could
// probably be drastically improved with a gausian filter tuned to each 
// competition location, but that's more work. 
bool seesDisc() {
    return lineSensor.get_value() < 2400;
}

/**
 * The task callback function used to keep track of the number of discs in 
 * the robot. This loops to keep track of the number of discs in the robot. 
 */
void task_function(void* data) {
    while(true) {
        printf("Number of discs: %d\n", int(discCount));
        printf("    State %d\n", state);
        printf("    Line Sensor Value: %d\n", lineSensor.get_value());
        printf("    Pros err: %d\n", errno);


        bool seeingDisc = seesDisc();
        printf("    seeingDisc = %i\n", seeingDisc);
        
        // If we have not yet seen a disc and we just started seeing one,
        // set the appropriate state depending on the intake's direction
        if(state == State::NO_DISC && seeingDisc) {
            printf("    Started Seeing Disc\n");
            if(intake::intaking()) {
                printf("    state = State::DISC_INTAKE\n");
                state = State::DISC_INTAKE;
            }
            else if(intake::outtaking()) {
                printf("    state = State::DISC_OUTTAKE\n");
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
            discCount++;
        }
        // Similarly, if we saw a disc when the intake started outtaking, and we
        // stop seeing the disc while it is still outtaking, it must have been
        // ejected from the robot. 
        else if(
            state == State::DISC_OUTTAKE && intake::outtaking() // Same direction
            && !seeingDisc                                      // Stopped Seeing
        ) {
            discCount--;
        }

        // Blow up the terminal if we somehow got a negative disc count; 
        if(discCount < 0) {
            printf("ERROR: Disc count somehow negative!!!!\n");
        }

        pros::delay(10);
    }
}

// Initializes the subsystem by starting the task.
void initialize() {
    printf("Initializing Disc Counter Subsystem: ");
    pros::Task task(task_function, nullptr, "Disc Counter Task");
    printf("Done\n");
}

// Returns the number of discs in the robot currently.
int disc_count() {
    return discCount;
}

// Decrements the number of discs in the robot
void decrement() {
    discCount--;

    if(discCount < 0) {
        printf("ERROR: Disc count somehow negative!!!!\n");
    }
}

} // namespace disccounter