#include "subsystems/disccounter.hpp"

#include "main.h"
#include "subsystems/subsystems.hpp"

#include "ARMS/config.h"

#include <atomic>
#include <memory>

using namespace pros;

namespace disccounter {

// The line sensor being used for detecting the discs.
ADIAnalogIn lineSensor(INTAKE_LINE);

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
bool seeing_disc() {
    return lineSensor.get_value() < 2400;
}

void debug_screen() {
    pros::lcd::print(0, "Disc Counter Info:");
    pros::lcd::print(1, "Disc Count: %d", disc_count());
    pros::lcd::print(2, "Seeing Disc: %d", seeing_disc());
    pros::lcd::print(3, "Sensor Value: %d", lineSensor.get_value());
    
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
    pros::lcd::print(4, "State: %s", state_str);
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