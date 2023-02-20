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

void shoot(int count, double angle) {
	disklift::discLiftUp();
	pros::delay(500);
	disklift::discLiftHold();
	for (int i = 1; i < count; i++) {
		turret::goto_angle(angle, 400, true);
		flywheel::wait_until_at_speed();
		flywheel::fire();
		flywheel::wait_until_fired();
		flywheel::stopIndexer();
	}
	turret::goto_angle(angle, 400, true);
	flywheel::wait_until_at_speed();
	flywheel::fire();
	pros::delay(750);
	flywheel::stopIndexer();
	disklift::discLiftDown();
}

void matchAuto() {
	using namespace arms::chassis;
	
	// setup
	arms::odom::reset({0, 0}, 0.0); // start position
	flywheel::move(167);
	intake::toggle();
	deflector::toggle();
	deflector::toggle();
	intake::move(100);

	// intake first disk
    std::cout << "Fetching first disc" << std::endl;
	//move({15,0}, 70, arms::THRU);
	move({30, -0.5}, 50);
	pros::delay(500);

	// spin roller
    std::cout << "Spinning roller" << std::endl;
	move({17,0}, 50, arms::REVERSE);
	pros::delay(100);
	turn(90, 50);
	pros::delay(100);
	tank(-50,-50);
	pros::delay(1000);
	tank(0,0);
	roller::move(100);
	pros::delay(125);
	roller::move(0);
	
	// shoot disks
    std::cout << "Shooting preloads" << std::endl;
	arms::odom::reset({18,-3},90);
	turret::goto_angle(-12, 400, true);
	move({18,3}, 50);
	pros::delay(500);
	shoot(3, -12);
	//pros::delay(500);

	turn(35, 60);
	pros::delay(100);
	move({21,8},50);
	pros::delay(100);

	
    std::cout << "Fetching disc 4" << std::endl;
	flywheel::move(135);
    turn(150, 60);
	pros::delay(100);
    move({0,25}, 50);
	pros::delay(100);
    
    std::cout << "Fetching disc 5" << std::endl;
    turn(50, 60);
	turret::goto_angle(5, 400, true);
	pros::delay(100);
    move({3,33}, 50);
	pros::delay(500);
	move(-5, 50, arms::REVERSE & arms::THRU);
	turn(50, 70, arms::ASYNC);
	pros::delay(1500);
	shoot(3,5);
	
    std::cout << "Shooting discs 4, 5" << std::endl;
	flywheel::move(140);
	turn(150, 60);
	pros::delay(500);
	move({-9,39},50);
	pros::delay(500);
	turn(60, 60);
	pros::delay(500);
	move({-7,44},60);

    std::cout << "Fetching discs 6" << std::endl;
	turret::goto_angle(-75, 400, true);
	pros::delay(500);
    turn(140, 60);
	pros::delay(500);
	move({-20,50}, 50);
	pros::delay(500);
	turn(110, 60);
	pros::delay(500);
	shoot(3, -75);

    std::cout << "Fetching discs 7" << std::endl;
	turret::goto_angle(0, 400, true);
	flywheel::move(165);
	move({-20,37},50, arms::REVERSE);
	pros::delay(500);
	turn(175, 60);
	pros::delay(500);
	move({-24,37},50);
	turn(220, 60);
	move({-26,34}, 50);
	turn(240, 60);
	move({-28,30});
	pros::delay(500);

    std::cout << "Shooting discs 6, 7" << std::endl;
    /* TODO: Implement this when the intake gets fixed */
	turn(45, 60);
	pros::delay(1000);
	shoot(3,0);
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
	shoot(2,-7);

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