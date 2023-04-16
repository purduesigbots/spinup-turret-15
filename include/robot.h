#pragma once

// Use these to compare
#define SILVER  1
#define GOLD    2

// Change this define to change which turret bot you are building.
#define BOT (SILVER)

#if BOT == SILVER
    #include "ARMS/config_silver.h"
#elif BOT == GOLD
    #include "ARMS/config_gold.h"
#else
	#error "INVALID BOT TYPE!!!! Set BOT to either SILVER or GOLD in robot.h"
#endif