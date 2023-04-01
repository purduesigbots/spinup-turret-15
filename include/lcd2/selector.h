#ifndef _LCD2_SELECTOR_H_
#define _LCD2_SELECTOR_H_

#include "display/lvgl.h"

namespace lcd2::selector {
void init(lv_obj_t* page, const char** autons, int default_auton);
/**
 * @brief Get the currently selected autonomous.
 * 
 * @return Selected auton number. Red autos are positive, blue autos are negative, and skills is 0.
 */
int get_auton();
}

#endif