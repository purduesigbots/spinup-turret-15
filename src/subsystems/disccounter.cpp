#include "subsystems/disccounter.hpp"

#include "main.h"
#include "subsystems/subsystems.hpp"

using namespace pros;

namespace disccounter {

ADIAnalogIn lineSensor('f');

Task task;

int discCount = 0;

bool seesDisc() {
    return lineSensor.get_value() < 2000;
}

void task_function(void* data) {
    bool sawDisc = seesDisc();

    while(true) {
        bool seeingDisc = seesDisc();

        if(sawDisc == true && seeingDisc == false) {
            printf("Disc intook.\n");
            discCount++;
        }

        sawDisc = seeingDisc;

        pros::delay(10);
    }
}

void initialize() {
    task = Task(task_function);
}

int disc_count() {
    return discCount;
}

int decrement() {
    discCount--;

    if(discCount < 0)
        discCount = 0;
}

} // namespace disccounter