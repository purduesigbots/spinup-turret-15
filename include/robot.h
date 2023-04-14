#pragma once

// Use these to compare
#define SILVER  1
#define GOLD    2

// Change this define to change which turret bot you are building.
#define BOT (SILVER)

/**
*
* COMPILATION SANITY CHECK (DO NOT REMOVE)
*
*/
#if BOT == SILVER
	#warning "Building Sliver Bot"
    #include "ARMS/config_silver.h"
#elif BOT == GOLD
	#warning "Building Gold Bot"
    #include "ARMS/config_gold.h"
#else
	#error "INVALID BOT TYPE!!!! Set BOT to either SILVER or GOLD in robot.h"
#endif