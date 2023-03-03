#include "main.h"

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
#if 0
void shoot(int count, double angle) {
	turret::enable_vision_aim();
	disklift::lift_motor.move_voltage(12000);
	for (int i = 0; i < 125; i++) {
		turret::update();
		pros::delay(10);
	}
	disklift::lift_motor.move_voltage(6000);
	for (int i = 1; i < count; i++) {
		flywheel::wait_until_at_speed();
		flywheel::fire();
		flywheel::wait_until_fired();
		flywheel::stopIndexer();
	}
	intake::move(-100);
	flywheel::wait_until_at_speed();
	flywheel::fire();
	pros::delay(750);
	flywheel::stopIndexer();
	disklift::discLiftDown();
	turret::disable_vision_aim();
	intake::move(100);
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
	arms::odom::reset({18,-3},87);
	turret::goto_angle(-13, 250, true);
	turret::update();
	move({18,3}, 50);
	pros::delay(500);
	shoot(3, -13);
	turret::goto_angle(0, 250, true);
	turret::update();
	//pros::delay(500);

	flywheel::move(140);
	turn(30, 60);
	pros::delay(100);
	move({23,7},50);
	pros::delay(100);

	
    std::cout << "Fetching disc 4" << std::endl;
    turn(150, 60);
	pros::delay(100);
    move({0,20}, 50);
	pros::delay(100);
    
    std::cout << "Fetching disc 5" << std::endl;
    turn(65, 60);
	turret::goto_angle(0, 250, true);
	turret::update();
	pros::delay(100);
    move({3,28}, 50);
	pros::delay(500);
	move(-5, 50, arms::REVERSE & arms::THRU);
	pros::delay(500);
	shoot(3,0);
	
    std::cout << "Shooting discs 4, 5" << std::endl;
	flywheel::move(135);
	turn(150, 60);
	pros::delay(500);
	move({-10,30},50);
	pros::delay(500);
	turn(70, 60);
	pros::delay(500);
	move({-9,38},60);

    std::cout << "Fetching discs 6" << std::endl;
	turret::goto_angle(-75, 250, true);
	turret::update();
	pros::delay(500);
    turn(155, 60);
	pros::delay(500);
	move({-22,45}, 50);
	pros::delay(1000);
	turn(110, 60);
	pros::delay(500);
	shoot(3, -75);

    std::cout << "Fetching discs 7" << std::endl;
	flywheel::move(165);
	move({-21,35},50, arms::REVERSE);
	turret::goto_angle(0, 250, true);
	turret::update();
	pros::delay(500);
	turn(178, 60);
	pros::delay(500);
	move({-26,35},50);
	turn(217, 60);
	move({-28,33}, 50);
	turn(245, 60);
	move({-30,28});
	pros::delay(500);
    std::cout << "Shooting discs 6, 7" << std::endl;
    /* TODO: Implement this when the intake gets fixed */
	turn(55, 60);
	pros::delay(1000);
	shoot(3,0);
	deflector::toggle();
	flywheel::move(0);
}

void skillsAuto() {
	using namespace arms::chassis;
	
	// setup
	arms::odom::reset({0, 0}, 0.0); // start position
	flywheel::move(120);
	intake::toggle();
	deflector::toggle();
	deflector::toggle();
	intake::move(100);
	vision::set_vision_offset(false);

	// spin roller
    std::cout << "First 3 Stack" << std::endl;
	move({15,0}, 50);
	pros::delay(100);
	move({32,0},30);
	turret::goto_angle(-65, 250, true);
	turret::update();
	pros::delay(500);
	turn(-45, 60);
	pros::delay(500);
	shoot(3, -65);

	move({73,-43},50);
	pros::delay(500);
	turn(-95, 60);
	pros::delay(500);
	shoot(3, -65);

	flywheel::move(165);
	turn(-50, 60);
	pros::delay(500);
	move({88,-69},50);
	pros::delay(500);
	turn(90,60);
	pros::delay(500);
	tank(-50,-50);
	pros::delay(750);
	tank(0,0);
	pros::delay(100);
	arms::odom::reset({88,-81},90);
	turret::goto_angle(-45,250,true);
	turret::update();
	move({88,-76},50);
	pros::delay(500);
	turn(0, 60);
	pros::delay(500);
	move({125,-78},50);
	pros::delay(500);
	turn(135,60);
	pros::delay(500);
	move({116,-69},50);
	pros::delay(500);
	shoot(2,-45);

	flywheel::move(145);
	move({101,-50},30);
	turret::goto_angle(-50,250,true);
	turret::update();
	pros::delay(1000);
	intake::move(-100);
	move(105, 50);
	pros::delay(500);
	turn(-45, 60);

	// flywheel::move(120);
	// move({71,-32},50);
	// turret::goto_angle(-78,250,true);
	// turret::update();
	// pros::delay(500);
	// shoot(2,-78);
}
#endif 
extern "C" {

void subsystem_test() {
	printf("Intaking 3 discs:\n");
	
	intake::start(300);

	disccounter::expect(3, 10000);

	printf("Done\n");

	intake::stop();
}

void autonomous() {
	subsystem_test();

#if 0

	vision::set_vision_offset(true);
	roller::set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	
	switch (arms::selector::auton) {
		case 0:
			skillsAuto();
			break;
		default:
			matchAuto();
			break;
	}
	turret::goto_angle(0,250,true);
#endif
}
}