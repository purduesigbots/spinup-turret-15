#include "main.h"
#include "ARMS/config.h"
#include "comms/comms.hpp"
#include "pros/misc.h"
#include "subsystems/flywheel.hpp"
#include "subsystems/subsystems.hpp"
#include "subsystems/vision.hpp"

#define FLYWHEEL_GRAPHING false

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

	while (true) {
		lcd2::pages::print_line(0, 0, "General Info:");
		lcd2::pages::print_line(0, 1, "Robot: %s", BOT == GOLD ? "Goldilocks" : "Octavio \"Octane\" Silva");
		lcd2::pages::print_line(0, 2, " X: %2.4f, Y: %2.4f, H: %2.4f)", 
			arms::odom::getPosition().x, 
			arms::odom::getPosition().y, 
			arms::odom::getHeading());
		lcd2::pages::print_line(0, 3, " Turret Angle: %3.5f", turret::get_angle());
		lcd2::pages::print_line(0, 4, " Distance to goal: %2.4f", vision::get_distance());

		discLift::debug_screen();
		turret::debug_screen();
		flywheel::debug_screen();
		discCounter::debug_screen();
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
	printf("\nHELLO THERE");
	const char* autons[] = {AUTONS, ""};
	const char* pages[] = {"General", "DiscLift", "Turret", "Flywheel", "DiscCounter", ""};
	lcd2::lcd2_parameters parameters = {
		autons,
		DEFAULT,
		false,
		pages
	};
	lcd2::initialize(parameters);
	turret::initialize();
	sylib::initialize();
	discLift::home();
	arms::init();
	arms::odom::reset({0, 0}, 0.0); // start position
	pros::delay(500);
	flywheel::initialize();
	roller::init();
	discCounter::initialize();
	pros::Task screenTask(draw_screen, "Debug Daemon");
	printf("Done initializing!!!\n");
	vision::init();
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

pros::Controller master(CONTROLLER_MASTER);

void joystick() {
	while(true) {
		double axis1 = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
		double axis2 = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
		double axis4 = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
		double axis3 = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);

		printf("%f | %f | %f | %f\n", axis1, axis2, axis3, axis4);
		pros::delay(75);
	}
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
	
	/**
	*
	* LOCAL VARIABLES
	*
	*/

	//Controller print counter
	int counter = 0;
	//State variable: should be using vision aim
	bool use_vision = true; //Default to true--vision will enable on DL button press
	//State variable: should be using auto speed
	bool use_auto_speed = true;
	//State variable: is vision good
	bool vision_good = false;
	double manual_angle = 0.0;
	//Counter for disc lift intaking to avoid jamming
	int discLiftCounter = 0;
	//Driver's controller local variable
	// pros::Task jt(joystick);
	pros::Controller partner(CONTROLLER_PARTNER);

	//Further initialization business--competition + not competition
	// turret::goto_angle(0, 250, true);
	flywheel::set_auto_speed_mode(use_auto_speed);

	//Further initialization business--ONLY competition
	if(pros::competition::is_connected()){
		flywheel::start(115);
		deflector::up();
	}

	
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
				intake::start(100);
			} else {
				intake::start(-100);
			}
			discLiftCounter++;
		} else if (!master.get_digital(DIGITAL_L1)) {
			discLift::discLiftDown();
			turret::disable_vision_aim();
			turret::goto_angle(manual_angle,100,true);
		}

		/**
		 * TURRET ENDGAME CONTROLS
		 */
		int partner_x = partner.get_analog(ANALOG_RIGHT_X);
		int partner_y = partner.get_analog(ANALOG_RIGHT_Y);
		if (partner_x * partner_x + partner_y * partner_y > 7200) {
			manual_angle = atan2(partner_y, partner_x) - M_PI_2;
			manual_angle *= 180 * M_1_PI;
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
			flywheel::change_target_speed(1);
		}
		if (master.get_digital_new_press(DIGITAL_DOWN)) {
			flywheel::change_target_speed(-1);
		}
		if(counter % 50 == 0){
			master.print(1, 0, "Flywheel Speed: %3d", int(flywheel::target_speed()));
		}
		if (master.get_digital_new_press(DIGITAL_B)) { //Auto speed toggle
			flywheel::set_auto_speed_mode(!use_auto_speed);
			use_auto_speed = !use_auto_speed;
		}
		/**
		*
		* VISION CONTROLS
		*
		*/
		if (!vision::is_working()) {
			vision_good = false;
			if (counter % 50 == 10) {
				master.print(0, 0, "Vision Bad");
			}
		} else if (!vision_good && counter % 50 == 10) {
			master.clear_line(0);
			vision_good = true;
		}
		if(master.get_digital_new_press(DIGITAL_Y)){
			use_vision = !use_vision;
		}
		if (fabs(vision::get_error()) < 2.0 && counter % 10 == 5) {
			master.rumble("-");
		}

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

		#if FLYWHEEL_GRAPHING
			//Flywheel speed graphing utility prints
			printf("graph_data\n");
			printf("time (ms),f1 velocity (rpm),f2 velocity (rpm), "
				"target|%d,%.2f,%.2f,%.2f\n",
				pros::millis(), flywheel::current_speed(1),
				flywheel::current_speed(), flywheel::target_speed());
		#endif

		//LOOP DELAY
		pros::delay(20);
	}
}
