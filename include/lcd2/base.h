#ifndef _LCD2_BASE_H_
#define _LCD2_BASE_H_

namespace lcd2 {
/**
 * @brief Parameters used to initialize LCD2.
 */
struct lcd2_parameters {
    /**
     * @brief List of auton names, ending with an empty string.
     * To disable the selector, pass in an array with an empty string {""}.
     */
    const char** autons;
    /**
     * @brief The inital autonomous selected.
     * Red autos are positive, blue autos are negative, and skills is 0.
     */
    int default_auto;
    /**
     * @brief Set to true to enable log, and false to disable log.
     */
    bool enable_log;
    /**
     * @brief List of page titles, ending with an empty string.
     * If you want no pages, pass in an array with an empty string {""}.
     */
    const char** page_titles;
};
/**
 * @brief Create and initialize all of the views, as specified by the parameters.
 * 
 * @param parameters the parameters used to initialize LCD2.
 */
void initialize(lcd2_parameters parameters);
}

#endif