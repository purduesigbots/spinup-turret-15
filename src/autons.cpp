#include "main.h"

#include "subsystems.h"
#include "vision.h"
#include "ARMS/api.h"


/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */

extern "C" {
void autonomous() {
	using namespace arms::chassis;
	
	// setup
	arms::odom::reset({0, 0}, 0.0); // start position
	flywheel::move(150);
	intake::toggle();
	deflector::toggle();
	deflector::toggle();
	intake::move(100);

	// intake first disk
    std::cout << "Fetching first disc" << std::endl;
	move({32,0}, 70);
	pros::delay(500);

	// spin roller
    std::cout << "Spinning roller" << std::endl;
	move({20,0}, 50, arms::REVERSE);
	pros::delay(500);
	turn(90, 50);
	pros::delay(500);
	tank(-50,-50);
	pros::delay(1000);
	tank(0,0);
	roller::move(100);
	pros::delay(125);

	roller::move(0);
	
	// shoot disks
    std::cout << "Shooting preloads + corner disc into goal" << std::endl;
	arms::odom::reset({22,-3},90);
	move(13, 50);
	turret::move_angle(-15, 400);
	// disklift::discLiftUp();
	// pros::delay(250);
	// disklift::discLiftHold();
	// flywheel::fire();
	// flywheel::wait_until_fired();
	// flywheel::stopIndexer();
	// flywheel::wait_until_at_speed();
	// flywheel::fire();
	// flywheel::wait_until_fired();
	// flywheel::stopIndexer();
	// flywheel::wait_until_at_speed();
	// flywheel::fire();
	pros::delay(1000);
	flywheel::stopIndexer();

    std::cout << "Fetching disc 4" << std::endl;
    turn(180, 50);
    move({0, 12, 135}, 75);
    intake::move(100);
    move({-8, 24}, 80);
    
    std::cout << "Fetching disc 5" << std::endl;
    turn(45, 50);
    move({-6, 32}, 80);

    std::cout << "Shooting discs 4, 5" << std::endl;
    /* TODO: Implement this when the intake gets fixed */

    std::cout << "Fetching discs 6" << std::endl;
    // move({-8, 24}, 80, arms::REVERSE);
    // turn(135, 50);
    // move({-20, 24}, 80);

    std::cout << "Fetching discs 7" << std::endl;
    // turn(45, 50);
    // move({10, 32}, 80);

    std::cout << "Shooting discs 6, 7" << std::endl;
    /* TODO: Implement this when the intake gets fixed */



    
    flywheel::move(0);
}
}