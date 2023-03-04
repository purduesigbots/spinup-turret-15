#include "main.h"
#include "ARMS/config.h"
#include "ARMS/odom.h"
#include "pros/misc.h"
#include "comms/comms.hpp"
#include "LatPullDown/Oak_1_latency_compensator.hpp"
#include "subsystems/flywheel.hpp"
#include "subsystems/intake.hpp"
#include "subsystems/turret.hpp"
#include "vision.h"
#include "subsystems/subsystems.hpp"

#include <atomic>

using namespace pros;

// DO NOT REMOVE THIS. THIS IS A SANITY CHECK
#if BOT == SILVER
	#warning "Building Sliver Bot"
#elif BOT == GOLD
	#warning "Building Gold Bot"
#else 
	#error "INVALID BOT TYPE!!!! Set BOT to either SILVER or GOLD in robot.h"
#endif

std::map<uint8_t, int32_t> comms_data;

const int MIN_SCREEN_INDEX = 0;
const int MAX_SCREEN_INDEX = 3;
static std::atomic<int> screenIndex = 0;

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	screenIndex = 0;
}

void on_left_button() {
	printf("left button\n");
	if(screenIndex > MIN_SCREEN_INDEX) {
		screenIndex--;
	}
}

void on_right_button() {
	printf("right button\n");
	if(screenIndex < MAX_SCREEN_INDEX) {
		screenIndex++;
	}
}

void draw_screen() 
{
	pros::lcd::initialize();
	pros::lcd::set_background_color(LV_COLOR_BLACK);
	pros::lcd::set_text_color(LV_COLOR_WHITE);
	pros::lcd::register_btn0_cb(on_left_button);
	pros::lcd::register_btn2_cb(on_right_button);

	while(true) {
		pros::lcd::clear();

		if(screenIndex == 0) {
			pros::lcd::print(1, "X: %2.4f, Y: %2.4f, H: %2.4f)", 
				arms::odom::getPosition().x,
				arms::odom::getPosition().y,
				arms::odom::getHeading()
			);
			pros::lcd::print(2, "Turret Angle: %3.5f", turret::get_angle());
			pros::lcd::print(3, "Distance to goal: %2.4f", arms::odom::getDistanceError({0,0}));
			pros::lcd::print(4, "DiscLift Position %f", disclift::lift_motor.get_position());
			pros::lcd::print(5, "DL Temp: %f", disclift::lift_motor.get_temperature());
			pros::lcd::print(6, "DL Draw: %d", disclift::lift_motor.get_current_draw());
		}
		else if(screenIndex == 1) {
			disccounter::debug_screen();
		}
		else if(screenIndex == 2) {
			turret::debug_screen();
		}
		else if(screenIndex = 3) {
			flywheel::debug_screen();
		}

		pros::delay(10);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	vision::init();
	turret::initialize();
	sylib::initialize();
	disclift::home();
	arms::init();
	arms::odom::reset({0, 0}, 0.0); // start position
	pros::delay(2000);
	flywheel::initialize();
	vision::init();
	Task vision(vision::task);
	roller::init();
	disccounter::initialize();

	Task screenTask(draw_screen, "Debug Daemon");

	printf("Done initializing!!!\n");
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

	turret::goto_angle(0, 400, true);
	vision::set_vision_offset(false);
	vision::start_vision();
	
	roller::set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	//flywheel::start(115);

	int counter = 0;
	bool indexer_wait = false;
	bool use_vision = true;
	bool vision_good = false;

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
		

		if (master.get_digital_new_press(DIGITAL_L2)) { // Disc lift
			discLiftCounter = 0; 
			if (use_vision) {
				turret::enable_vision_aim();
			}
    	} 
		if (master.get_digital(DIGITAL_L2) && !master.get_digital(DIGITAL_L1)) {
			disclift::discLiftUp();
			if(discLiftCounter < 10){
				intake::start(1000);
			} else{
				intake::stop();
			}
			discLiftCounter++;
		} else if (!master.get_digital(DIGITAL_L1)){
			disclift::discLiftDown();
			turret::disable_vision_aim();
		}
	
		if (master.get_digital_new_press(DIGITAL_LEFT)){
			deflector::toggle();
		}
		if (master.get_digital_new_press(DIGITAL_RIGHT)){
			std::cout << "Launching Endgame" << std::endl;
			endgame::deploy();
		}
		if (master.get_digital_new_press(DIGITAL_B)){
			intake::toggle_arm();
		}
		
		if(master.get_digital_new_press(DIGITAL_L1)){
			disclift::calculatePos();
		}
		if (master.get_digital(DIGITAL_L1)){
			if (flywheel::at_speed()) {
				flywheel::fireControl_driver(true);
			} else {
				flywheel::fireControl_driver(false);
			}
			disclift::discLiftHold();
		} else {
			flywheel::fireControl_driver(false);
		}
		
		if (master.get_digital(DIGITAL_R1)) { // intake
			intake::start(100);
			roller::move(100);
		} else if (master.get_digital(DIGITAL_R2)) { // outake
			intake::start(-100);
			roller::move(-100);
		} else if (!master.get_digital(DIGITAL_L2)) { // idle
			intake::stop();
			roller::move(0);
		}

		// Flywheel control
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

		if (vision::vision_not_working()) {
			vision_good = false;
			if (counter % 5 == 0) {
				master.print(0,0,"Vision Bad");
			}
		} else if (!vision_good && counter % 5 == 0) {
			master.clear_line(0);
			vision_good = true;
		}

		if(master.get_digital_new_press(DIGITAL_X)){
			//indexer_wait = !indexer_wait;
			autonomous();
			// turret::toggle_vision_aim();
		}
		counter++;
		pros::delay(20);
	}
}

