#include "subsystems/deflector.hpp"

#include "api.h"
#include "subsystems/subsystems.hpp"

using namespace pros;

namespace deflector {

ADIDigitalOut deflector_piston(ext_adi_port_pair_t{8, 'c'});
bool state = true;

void toggle(){
    state = !state;
    deflector_piston.set_value(state);
}

} // namespace deflector
