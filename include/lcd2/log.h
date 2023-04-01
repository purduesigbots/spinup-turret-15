#ifndef _LCD2_LOG_H_
#define _LCD2_LOG_H_

#include "display/lvgl.h"
#include <string>

namespace lcd2::log {
void init(lv_obj_t* page);
/**
 * @brief Add a line of text to the log.
 * 
 * @param text Line of text you want added to the log.
 * @return true when text has been added successfully,
 * @return false when log has not been initialized.
 */
bool print(std::string text);
}

#endif