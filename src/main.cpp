#include "main.h"
#include "ARMS/config.h"
#include "ARMS/odom.h"
#include "pros/misc.h"
#include "subsystems.h"
#include "comms/comms.hpp"
#include "LatPullDown/Oak_1_latency_compensator.hpp"
#include "vision.h"
#include "subsystems/subsystems.hpp"

// DO NOT REMOVE THIS. THIS IS A SANITY CHECK
#if BOT == SILVER
	#warning "Building Sliver Bot"
#elif BOT == GOLD
	#warning "Building Gold Bot"
#else 
	#error "INVALID BOT TYPE!!!! Set BOT to either SILVER or GOLD in robot.h"
#endif

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
	Task disklift_home_task([](void){
		disklift::home();
	});

	arms::init();
	arms::odom::reset({0, 0}, 0.0); // start position
	pros::lcd::initialize();
	pros::lcd::clear();
	pros::lcd::set_background_color(LV_COLOR_BLACK);
	pros::lcd::set_text_color(LV_COLOR_WHITE);
	//pros::delay(2000);
	Task flywheel(flywheel::task);
	vision::init();
	Task vision(vision::task);
	turret::initialize();

	// pros::lcd::register_btn1_cb(on_center_button);

	roller::init();
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
	
	roller::set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

	int counter = 0;
	bool indexer_wait = false;

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
		pros::lcd::print(1, "Pose: %2.4f, %2.4f, %2.4f)", 
			arms::odom::getPosition().x,
			arms::odom::getPosition().y,
			arms::odom::getHeading()
		);
		pros::lcd::print(2, "Turret Angle: %3.5f", turret::get_angle());
		// pros::lcd::print(3, "Goal Gamma: %2.4f", vision::get_goal_gamma());
		pros::lcd::print(4, "DiscLift Position %f", disklift::lift_motor.get_position());
		pros::lcd::print(5, "DL Temp: %f", disklift::lift_motor.get_temperature());
		pros::lcd::print(6, "DL Draw: %d", disklift::lift_motor.get_current_draw());

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
			std::cout << "Launching Endgame" << std::endl;
			endgame::launch();
		}
		if (master.get_digital_new_press(DIGITAL_B)){
			intake::toggle();
		}
		
		

		if(master.get_digital_new_press(DIGITAL_L1)){
			disklift::calculatePos();
		}
		if (master.get_digital(DIGITAL_L1)){
			if (/* !indexer_wait || */ flywheel::at_speed()) {
				flywheel::fire();
			} else {
				flywheel::stopIndexer();
			}
			disklift::discLiftHold();
		} else {
			flywheel::stopIndexer();
		}
		
		if (master.get_digital(DIGITAL_R1)) { // intake
			intake::move(100);
			roller::move(100);
		} else if (master.get_digital(DIGITAL_R2)) { // outake
			intake::move(-100);
			roller::move(-100);
		} else if (!master.get_digital(DIGITAL_L2)) { // idle
			intake::move(0);
			roller::move(0);
		}

		// Flywheel control
		if (master.get_digital_new_press(DIGITAL_A)) {
			if (flywheel::speed == 0) {
				flywheel::move(120); // max = 200
			} else {
				flywheel::move(0);
			}
		}

		if (master.get_digital_new_press(DIGITAL_UP)) {
			flywheel::move(flywheel::speed + 1);
			master.print(1, 1, "Flywheel speed: %.1f", flywheel::speed);
		}

		if (master.get_digital_new_press(DIGITAL_DOWN)) {
			flywheel::move(flywheel::speed - 1);
			master.print(1, 1, "Flywheel speed: %.1f", flywheel::speed);
		}

		if(master.get_digital_new_press(DIGITAL_X)){
			//indexer_wait = !indexer_wait;
			autonomous();
			//turret::toggle_vision_aim();
		}
		turret::update();
		pros::delay(20);
	}
}

