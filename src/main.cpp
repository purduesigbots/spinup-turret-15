#include "main.h"
#include "ARMS/config.h"
#include "ARMS/odom.h"
#include "pros/misc.h"
#include "subsystems.h"
#include "comms/comms.hpp"
#include "LatPullDown/Oak_1_latency_compensator.hpp"
#include "vision.h"

std::map<uint8_t, int32_t> comms_data;
/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	sylib::initialize();
	turret::home();

	arms::init();
	arms::odom::reset({0, 0}, 0.0); // start position
	pros::delay(2000);
	Task flywheel(flywheel::task);
	//vision::init();
	//Task vision(vision::task);


	pros::lcd::initialize();
	pros::lcd::clear();
	pros::lcd::set_text(1, "Hello PROS User!");
	pros::lcd::set_background_color(LV_COLOR_BLACK);
	pros::lcd::set_text_color(LV_COLOR_WHITE);

	pros::lcd::register_btn1_cb(on_center_button);

	roller::init();
	disklift::home();
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
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
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	using namespace arms::chassis;

	int counter = 0;

	// move(30, 70);
	// move(-4, 50, arms::REVERSE);
	// turn(89);
	// move(-6, 50, arms::REVERSE);
	// move(5, 50, arms::REVERSE);
	// turn(51, arms::RELATIVE);
	// move(36, 70);
	// turn(-90, arms::RELATIVE);
	// move(5, 50);

	// flywheel::move(90);

	int discLiftCounter = 0;
	bool prevDLButton = false;

	while (true) {

		int left = master.get_analog(ANALOG_LEFT_Y);
		int right = master.get_analog(ANALOG_RIGHT_X);
		arcade(left, right);
		pros::lcd::print(1, "X: %f", arms::odom::getPosition().x);
		pros::lcd::print(2, "Y: %f", arms::odom::getPosition().y);
		pros::lcd::print(3, "Heading: %f", arms::odom::getHeading());
		
		if (master.get_digital_new_press(DIGITAL_L2)) { // Disc lift
			discLiftCounter = 0; 
    	} 
		if (master.get_digital(DIGITAL_L2) && !master.get_digital(DIGITAL_L1)) {
			disklift::discLiftUp();
			if(discLiftCounter < 10){
				intake::move(100);
			} else{
				intake::move(0);
			}
			discLiftCounter++;
		} else if (!master.get_digital(DIGITAL_L1)){
			disklift::discLiftDown();
		}
	
		if (master.get_digital_new_press(DIGITAL_LEFT)){
			deflector::toggle();
		}
		if (master.get_digital_new_press(DIGITAL_RIGHT)){
			endgame::launch();
		}
		if (master.get_digital_new_press(DIGITAL_B)){
			intake::toggle();
		}
		
		

		if(master.get_digital_new_press(DIGITAL_L1)){
			disklift::calculatePos();
		}
		if (master.get_digital(DIGITAL_L1)){
			flywheel::fire();
			disklift::discLiftHold();
		} else {
			flywheel::stopIndexer();
		}
		
		if (master.get_digital(DIGITAL_R1)) { // intake
			intake::move(100);
			roller::move(100);
      		printf("pressed");
		} else if (master.get_digital(DIGITAL_R2)) { // outake
			intake::move(-100);
			roller::move(-100);
		} else if (!master.get_digital(DIGITAL_L2)) { // idle
			intake::move(0);
			roller::move(0);
		}


    	// if (master.get_digital(DIGITAL_A)) { // Turret Left
		// 	turret::move(127);
		// } else if (master.get_digital(DIGITAL_X)) { // Turret Right
		// 	turret::move(-127);
		// } else { // idle
		// 	turret::move(0);
		// }


		// Flywheel control
		if (master.get_digital_new_press(DIGITAL_A)) {
			if (flywheel::speed == 0) {
				flywheel::move(90); // max = 200
			} else {
				flywheel::move(0);
			}
		}

		if (master.get_digital_new_press(DIGITAL_UP)) {
			flywheel::move(flywheel::speed + 10);
			master.print(1, 1, "Flywheel speed: %.1f", flywheel::speed);
		}

		if (master.get_digital_new_press(DIGITAL_DOWN)) {
			flywheel::move(flywheel::speed - 5);
			master.print(1, 1, "Flywheel speed: %.1f", flywheel::speed);
		}

		if(!pros::competition::is_connected() && master.get_digital_new_press(DIGITAL_X)){
			autonomous();
		}

		pros::delay(20);
	}
}

