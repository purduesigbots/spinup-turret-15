#include "main.h"
#include "ARMS/config.h"
#include "comms/comms.hpp"
#include "subsystems/subsystems.hpp"

/**
*
* COMPILATION SANITY CHECK (DO NOT REMOVE)
*
*/
#if BOT == SILVER
	#warning "Building Sliver Bot"
#elif BOT == GOLD
	#warning "Building Gold Bot"
#else
	#error "INVALID BOT TYPE!!!! Set BOT to either SILVER or GOLD in robot.h"
#endif

/**
*
* DEBUG SCREEN
*
*/

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

/**
*
* MAIN ROBOT METHODS
*
*/

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	vision::init();
	printf("\nHELLO THERE");
	turret::initialize();
	sylib::initialize();
	discLift::home();
	arms::init();
	arms::odom::reset({0, 0}, 0.0); // start position
	pros::delay(2000);
	flywheel::initialize();
	roller::init();
	discCounter::initialize();
	pros::Task screenTask(draw_screen, "Debug Daemon");
	printf("Done initializing!!!\n");
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
	//Should we do the arms autonomous selector here?
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
	//Should we do the arms autonomous selector here?
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {

	//Further initialization business--competition + not competition
	// turret::goto_angle(0, 250, true);
	// vision::set_vision_offset(240);
	// turret::disable_vision_aim();

	//Further initialization business--ONLY competition
	if(pros::competition::is_connected()){
		flywheel::start(115);
		deflector::up();
	}

	/**
	*
	* LOCAL VARIABLES
	*
	*/

	//Controller print counter
	int counter = 0;
	//State variable: should be using vision aim
	bool use_vision = false;
	//State variable: is vision good
	bool vision_good = false;
	//Counter for disc lift intaking to avoid jamming
	int discLiftCounter = 0;
	//Driver's controller local variable
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	
	/**
	*
	* DRIVER CONTROL LOOP
	*
	*/
	while (true) {
		//Joystick values
		int left = master.get_analog(ANALOG_LEFT_Y);
		int right = master.get_analog(ANALOG_RIGHT_X);
		//Chassis control
		arms::chassis::arcade(left, right);

		/*
		*
		* DISC LIFT CONTROLS
		*
		*/
		if (master.get_digital_new_press(DIGITAL_L2)) {
			discLiftCounter = 0;
			if (use_vision) {
				turret::enable_vision_aim();
			}
		}
		if (master.get_digital(DIGITAL_L2) && !master.get_digital(DIGITAL_L1)) {
			discLift::discLiftUp();
			if (discLiftCounter < 10) {
				intake::start(1000);
			} else {
				intake::stop();
			}
			discLiftCounter++;
		} else if (!master.get_digital(DIGITAL_L1)) {
			discLift::discLiftDown();
			turret::disable_vision_aim();
			turret::goto_angle(0,100,true);
		}

		/**
		*
		* DEFLECTOR CONTROLS
		*
		*/
		if (master.get_digital_new_press(DIGITAL_LEFT)) {
			deflector::toggle();
		}

		/**
		*
		* ENDGAME CONTROLS
		*
		*/
		if (master.get_digital_new_press(DIGITAL_RIGHT)) {
			std::cout << "Launching Endgame" << std::endl;
			endgame::deploy();
		}

		/**
		*
		* INTAKE, ROLLER CONTROLS (grouped because they are on the same button)
		*
		*/
		if (master.get_digital(DIGITAL_R1)) { // intake
			intake::start(100);
			roller::move(65);
		} else if (master.get_digital(DIGITAL_R2)) { // outake
			intake::start(-100);
			roller::move(-65);
		} else if (!master.get_digital(DIGITAL_L2)) { // idle
			intake::stop();
			roller::move(0);
		}
		if (master.get_digital_new_press(DIGITAL_B)) { //intake arm
			intake::toggle_arm();
		}

		/**
		*
		* FLYWHEEL/INDEXER CONTROLS
		*
		*/
		if (master.get_digital(DIGITAL_L1)) {
			if (flywheel::at_speed()) {
				flywheel::fireControl_driver(true);
			} else {
				//set to false for rpm babysitter
				flywheel::fireControl_driver(false);
			}
			discLift::discLiftHold();
		} else {
			flywheel::fireControl_driver(false);
		}
		if (master.get_digital_new_press(DIGITAL_A)) {
			flywheel::toggle(120);
		}
		if (master.get_digital_new_press(DIGITAL_UP)) {
			flywheel::change_target_speed(10);
			master.print(1, 1, "Flywheel speed: %.1f", flywheel::target_speed());
		}
		if (master.get_digital_new_press(DIGITAL_DOWN)) {
			flywheel::change_target_speed(-5);
			master.print(1, 1, "Flywheel speed: %.1f", flywheel::target_speed());
		}
		/**
		*
		* VISION CONTROLS
		*
		*/
		// if (vision::vision_not_working()) {
		// 	vision_good = false;
		// 	if (counter % 5 == 0) {
		// 		master.print(0, 0, "Vision Bad");
		// 	}
		// } else if (!vision_good && counter % 5 == 0) {
		// 	master.clear_line(0);
		// 	vision_good = true;
		// }
		// if(master.get_digital_new_press(DIGITAL_Y)){
		// 	use_vision = !use_vision;
		// }

		/**
		*
		* MISC CONTROLS AND BOOKKEEPING
		*
		*/
		if (master.get_digital_new_press(DIGITAL_X)) {
			//Run auto if competition not connected
			if(!pros::competition::is_connected()){
				autonomous();
			}
		}
		
		//Increment controller printing counter
		counter++;

		//Flywheel speed graphing utility prints
		// printf("graph_data\n");
		// printf("time (ms),f1 velocity (rpm),f2 velocity (rpm), "
		//        "target|%d,%.2f,%.2f,%.2f\n",
		//        pros::millis(), flywheel::current_speed(1),
		//        flywheel::current_speed(), flywheel::target_speed());

		//LOOP DELAY
		pros::delay(20);
	}
}
