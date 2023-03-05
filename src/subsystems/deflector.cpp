#include "subsystems/deflector.hpp"

#include "ARMS/config.h"

#include "api.h"
#include "subsystems/subsystems.hpp"


using namespace pros;

namespace deflector {

ADIDigitalOut deflector_piston(DEFLECTOR_PISTON);
bool isUp = true;
bool state = false;

void up() {
    if(!isUp) {
        toggle();
    }
}

void down() {
    if(isUp) {
        toggle();
    }
}

void toggle(){
    state = !state;
    isUp = !isUp;
    deflector_piston.set_value(state);
}

bool is_up() {
    return isUp;
}

} // namespace deflector
