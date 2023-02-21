#include "subsystems/endgame.hpp"

#include "main.h"

using namespace pros;

namespace endgame {

ADIDigitalOut piston('f');
bool is_deployed = false;

void deploy() {
    is_deployed = true;
    piston.set_value(true);
    std::cout << "Endgame launched" << std::endl;
}

} // namespace endgame