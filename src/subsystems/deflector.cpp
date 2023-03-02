#include "subsystems/deflector.hpp"

#include "api.h"
#include "subsystems/subsystems.hpp"

using namespace pros;

namespace deflector {

ADIDigitalOut deflector_piston(ext_adi_port_pair_t{8, 'c'});
bool isUp = false;
bool state = true;

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