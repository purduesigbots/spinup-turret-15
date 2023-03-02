#ifndef _ARMS_CONFIG_H_
#define _ARMS_CONFIG_H_

#include "ARMS/api.h"
#include "main.h"
#include "subsystems.h"
#include <initializer_list>

namespace arms {

#if BOT == GOLD
  #include "ARMS/config_gold.h"
#elif BOT == SILVER
  #include "ARMS/config_silver.h"
#endif


// Initializer
inline void init() {
  chassis::init({LEFT_MOTORS}, {RIGHT_MOTORS}, GEARSET, SLEW_STEP,
                LINEAR_EXIT_ERROR, ANGULAR_EXIT_ERROR, SETTLE_THRESH_LINEAR,
                SETTLE_THRESH_ANGULAR, SETTLE_TIME);

  odom::init(ODOM_DEBUG, ENCODER_TYPE, {ENCODER_PORTS}, EXPANDER_PORT, IMU_PORT,
             TRACK_WIDTH, MIDDLE_DISTANCE, TPI, MIDDLE_TPI);

  pid::init(LINEAR_KP, LINEAR_KI, LINEAR_KD, ANGULAR_KP, ANGULAR_KI, ANGULAR_KD,
            TRACKING_KP, MIN_ERROR, LEAD_PCT);

  const char *b[] = {AUTONS, ""};
  selector::init(HUE, DEFAULT, b);
}

} // namespace arms

#endif
