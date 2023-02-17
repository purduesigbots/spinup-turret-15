#include "main.h"

#include "subsystems.h"
#include "vision.h"
#include "ARMS/api.h"

#include "subsystems/subsystems.hpp"

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

void shoot2() {
	disklift::discLiftUp();
	pros::delay(500);
	disklift::discLiftHold();
	flywheel::fire();
	flywheel::wait_until_fired();
	flywheel::stopIndexer();
	flywheel::wait_until_at_speed();
	pros::delay(250);
	flywheel::fire();
	flywheel::wait_until_fired();
	pros::delay(250);
	flywheel::stopIndexer();
	disklift::discLiftDown();
}

void matchAuto() {
	using namespace arms::chassis;
	
	// setup
	arms::odom::reset({0, 0}, 0.0); // start position
	flywheel::move(140);
	intake::toggle();
	deflector::toggle();
	deflector::toggle();
	intake::move(100);

	// intake first disk
    // std::cout << "Fetching first disc" << std::endl;
	// move({32,0}, 70);
	// pros::delay(500);

	// spin roller
    std::cout << "Spinning roller" << std::endl;
	move({17.3,0}, 50);
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
    std::cout << "Shooting preloads" << std::endl;
	arms::odom::reset({22,-3},90);
	move(13, 50);
	////turret::move_angle(-6, 400);
	shoot2();
	//pros::delay(500);

    std::cout << "Fetching disc 4" << std::endl;
	flywheel::move(120);
    turn(145, 60, arms::ASYNC);
	pros::delay(1500);
    move(21, 50);
	pros::delay(1000);
	pros::delay(500);
    
    std::cout << "Fetching disc 5" << std::endl;
    turn(70, 60, arms::ASYNC);
	pros::delay(3000);
    move(7, 50);
	pros::delay(1000);
	pros::delay(1000);
	////turret::move_angle(0, 400);
	shoot2();
	//pros::delay(500);

    std::cout << "Shooting discs 4, 5" << std::endl;
    /* TODO: Implement this when the intake gets fixed */
	turn(155, 60, arms::ASYNC);
	pros::delay(1500);
	move(18,50);
	pros::delay(1000);
	turn(70, 60, arms::ASYNC);
	pros::delay(1500);
	move(7, 50);
	pros::delay(1500);
	////turret::move_angle(-4, 400);
	shoot2();

    std::cout << "Fetching discs 6" << std::endl;
    // turn(155, 60, arms::ASYNC);
	// pros::delay(1500);
	// move(15, 50);
	// pros::delay(500);

    std::cout << "Fetching discs 7" << std::endl;
    // turn(105, 60, arms::ASYNC);
	// pros::delay(1500);
	// tank(-75,-75);
	// pros::delay(1300);
	// tank(-25,-25);
	// pros::delay(1000);
	// tank(0,0);
	// pros::delay(500);
	// arms::odom::reset({0,0},90);
	// move(13, 50);
	// pros::delay(500);
	// turn(160, 60);
	// pros::delay(2500);
	// move(20, 50);
	// pros::delay(500);

    std::cout << "Shooting discs 6, 7" << std::endl;
    /* TODO: Implement this when the intake gets fixed */
    flywheel::move(0);
}

void skillsAuto() {
	using namespace arms::chassis;
	
	// setup
	arms::odom::reset({0, 0}, 0.0); // start position
	flywheel::move(140);
	intake::toggle();
	deflector::toggle();
	deflector::toggle();
	intake::move(100);

	// spin roller
    std::cout << "Spinning roller" << std::endl;
	move({17.3,0}, 50);
	pros::delay(500);
	turn(90, 50);
	pros::delay(500);
	tank(-50,-50);
	pros::delay(1000);
	tank(0,0);
	roller::move(100);
	pros::delay(150);

	roller::move(0);
	
	// shoot disks
    std::cout << "Shooting preloads + corner disc into goal" << std::endl;
	arms::odom::reset({22,-3},90);
	move(13, 50);
	//turret::move_angle(-6, 400);
	shoot2();

	turn(15, 60, arms::ASYNC);
	pros::delay(3000);
	move(18, 50);
	pros::delay(500);
	turn(185, 60, arms::ASYNC);
	pros::delay(3000);

	tank(-50,-50);
	pros::delay(1000);
	tank(0,0);
	roller::move(100);
	pros::delay(150);
	roller::move(0);
	arms::odom::reset({0,0},0);

	move(18, 50);
	turn(-45, 60, arms::ASYNC);
	pros::delay(3000);
	endgame::launch();
	pros::delay(500);
}

extern "C" {
void autonomous() {
	roller::set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	
	switch (arms::selector::auton) {
		case 0:
			skillsAuto();
			break;
		default:
			matchAuto();
			break;
	}
}
}