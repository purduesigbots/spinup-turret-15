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
	turret::toggle_vision_aim();
	disklift::lift_motor.move_voltage(12000);
	pros::delay(1000);
	disklift::lift_motor.move_voltage(6000);
	for (int i = 1; i < count; i++) {
		flywheel::wait_until_at_speed();
		flywheel::fire();
		flywheel::wait_until_fired();
		flywheel::stopIndexer();
	}
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
	flywheel::move(120);
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
	pros::delay(100);
	roller::move(0);
	
	// shoot disks
    std::cout << "Shooting preloads" << std::endl;
	arms::odom::reset({18,-3},90);
	turret::goto_angle(-13, 400, true);
	flywheel::move(167);
	turret::update();
	move({18,3}, 50);
	pros::delay(500);
	shoot(3, -13);
	turret::goto_angle(0, 250, true);
	turret::update();
	//pros::delay(500);

	flywheel::move(90);
	turn(30, 60);
	pros::delay(100);
	move({23,7},50);
	pros::delay(100);

	
    std::cout << "Fetching disc 4" << std::endl;
    turn(142, 60);
	pros::delay(100);
    move({0,23}, 50);
	pros::delay(100);
    
    std::cout << "Fetching disc 5" << std::endl;
    turn(50, 60);
	turret::goto_angle(4, 400, true);
	flywheel::move(135);
	turret::update();
	pros::delay(100);
    move({4,30}, 50);
	pros::delay(500);
	move(-5, 50, arms::REVERSE & arms::THRU);
	turn(60, 70, arms::ASYNC);
	pros::delay(1500);
	shoot(3,4);
	
    std::cout << "Shooting discs 4, 5" << std::endl;
	flywheel::move(90);
	turn(145, 60);
	pros::delay(500);
	move({-8,33},50);
	pros::delay(500);
	turn(70, 60);
	pros::delay(500);
	move({-6,41},60);

    std::cout << "Fetching discs 6" << std::endl;
	turret::goto_angle(-75, 400, true);
	flywheel::move(135);
	turret::update();
	pros::delay(500);
    turn(155, 60);
	pros::delay(500);
	move({-20,47}, 50);
	pros::delay(500);
	turn(118, 60);
	pros::delay(500);
	shoot(3, -75);
	return;

    std::cout << "Fetching discs 7" << std::endl;
	flywheel::move(120);
	move({-24,34},50, arms::REVERSE);
	turret::goto_angle(0, 400, true);
	turret::update();
	pros::delay(500);
	turn(175, 60);
	pros::delay(500);
	move({-28,34},50);
	turn(225, 60);
	move({-30,30}, 50);
	turn(255, 60);
	move({-30,27});
	pros::delay(500);
    std::cout << "Shooting discs 6, 7" << std::endl;
    /* TODO: Implement this when the intake gets fixed */
	flywheel::move(165);
	turn(55, 60);
	pros::delay(1000);
	shoot(3,0);
    flywheel::move(0);
}

void skillsAuto() {
	using namespace arms::chassis;
	
	// setup
	arms::odom::reset({0, 0}, 0.0); // start position
	flywheel::move(125);
	intake::toggle();
	deflector::toggle();
	deflector::toggle();
	intake::move(100);

	// spin roller
    std::cout << "First 3 Stack" << std::endl;
	move({15,0}, 50);
	pros::delay(100);
	move({32,0},30);
	turret::goto_angle(-65, 400, true);
	turret::update();
	turn(-50, 60);
	pros::delay(500);
	shoot(3, -65);

	flywheel::move(165);
	move({72,-48},50);
	pros::delay(500);
	turn(-110, 60);
	pros::delay(500);
	shoot(3, -65);

	turret::goto_angle(-45,400,true);
	turret::update();
	turn(-45, 60);
	pros::delay(500);
	move({98,-79},50);
	pros::delay(500);
	turn(0, 60);
	pros::delay(500);
	move({115,-79},50);
	pros::delay(500);
	turn(135,60);
	pros::delay(500);
	move({112,-74},50);
	pros::delay(500);
	shoot(2,-45);
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
	turret::goto_angle(0,400,true);
}
}