#include "main.h"
#include "subsystems/subsystems.hpp"
#include "ARMS/api.h"

namespace{ //Anonymous namespace for private data and methods
    /**
    *
    * PRIVATE DATA
    *
    */

    const int MIN_SCREEN_INDEX = 0;
    const int MAX_SCREEN_INDEX = 4;
    static std::atomic<int> screenIndex = 0;
}

/**
*
* PUBLIC (not namespaced) METHODS
*
*/

/**
 * A callback function for LLEMU's center button.
 * Resets screen screen index to zero.
 */
void on_center_button() {
	screenIndex = 0;
}

/**
* A callback function for LLEMU's left button.
* Decrements screen index.
*/
void on_left_button() {
	printf("left button\n");
	if (screenIndex > MIN_SCREEN_INDEX) {
		screenIndex--;
	}
}

/**
* A callback function for LLEMU's right button.
* Increments screen index.
*/
void on_right_button() {
	printf("right button\n");
	if (screenIndex < MAX_SCREEN_INDEX) {
		screenIndex++;
	}
}

/**
* Renders debug screens to LLEMU
*/
void draw_screen() {
	pros::lcd::initialize();
	pros::lcd::set_background_color(LV_COLOR_BLACK);
	pros::lcd::set_text_color(LV_COLOR_WHITE);
	pros::lcd::register_btn0_cb(on_left_button);
	pros::lcd::register_btn2_cb(on_right_button);

	while (true) {
		pros::lcd::clear();
		if (screenIndex == 0) {
			pros::lcd::print(0, "General Info:");
			pros::lcd::print(1, "Robot: %s", BOT == GOLD ? "Goldilocks" : "Octavio \"Octane\" Silva");
			pros::lcd::print(2, " X: %2.4f, Y: %2.4f, H: %2.4f)", 
				arms::odom::getPosition().x, 
				arms::odom::getPosition().y, 
				arms::odom::getHeading());
			pros::lcd::print(3, " Turret Angle: %3.5f", turret::get_angle());
			pros::lcd::print(4, " Distance to goal: %2.4f", arms::odom::getDistanceError({0, 0}));
			
		} else if (screenIndex == 1) {
			discLift::debug_screen();
		} else if (screenIndex == 2) {
			turret::debug_screen();
		} else if (screenIndex == 3) {
			flywheel::debug_screen();
		} else if (screenIndex == 4){
			discCounter::debug_screen();
		}
		pros::delay(10);
	}
}