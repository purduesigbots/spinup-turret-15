#pragma once
#include "robot.h"

// This header is just for convienence. Includes all of the subsystems

#include "subsystems/deflector.hpp"
#include "subsystems/discCounter.hpp"
#include "subsystems/discLift.hpp"
#include "subsystems/endgame.hpp"
#include "subsystems/flywheel.hpp"
#include "subsystems/intake.hpp"
#include "subsystems/roller.hpp"
#include "subsystems/turret.hpp"
#include "subsystems/vision.hpp"

#if USING_BEN_PNEUMATICS
    #include "subsystems/pneumatics.hpp"
#endif