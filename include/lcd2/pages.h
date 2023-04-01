#ifndef _LCD2_PAGES_H_
#define _LCD2_PAGES_H_

#include "display/lvgl.h"
#include <string>

#define PAGE_LINES 8

namespace lcd2::pages {
typedef struct {
    lv_style_t style;
    lv_obj_t* lines[PAGE_LINES];
} page_ext_t;
void init(lv_obj_t* tabview, const char** titles);

/**
 * @brief Print a formatted line to a page.
 * 
 * @param page Page number, with first page being 0.
 * @param line Line number, from 0-7.
 * @param fmt Format string.
 * @param ... Additional arguments.
 * @return true when successful,
 * @return false when not successful
 */
bool print_line(int page, int line, const char* fmt, ...);
/**
 * @brief Set a line to a string of text.
 * 
 * @param page Page number, with first page being 0.
 * @param line Line number, from 0-7.
 * @param text Text that line will be set to.
 * @return true when successful,
 * @return false when not successful
 */
bool set_line(int page, int line, std::string text);

/**
 * @brief Clear a line on a page.
 * 
 * @param page Page number, with first page being 0.
 * @param line Line number, from 0-7.
 * @return true when successful,
 * @return false when not successful
 */
bool clear_line(int page, int line);
/**
 * @brief Clear all lines on a page.
 * 
 * @param page Page number, with first page being 0.
 * @return true when successful,
 * @return false when not successful
 */
bool clear_page(int page);
/**
 * @brief Clear all lines on all pages.
 * 
 * @return true when successful,
 * @return false when not successful
 */
bool clear_all();

/**
 * @brief Set the background color of a page.
 * 
 * @param page Page number, with first page being 0.
 * @param color Background color.
 * @return true when successful,
 * @return false when not successful
 */
bool set_background_color(int page, lv_color_t color);
/**
 * @brief Set the text color of a page.
 * 
 * @param page Page number, with first page being 0.
 * @param color Text color.
 * @return true when successful,
 * @return false when not successful
 */
bool set_text_color(int page, lv_color_t color);
}

#endif