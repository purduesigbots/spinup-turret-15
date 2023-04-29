#include "main.h"
#include "ARMS/api.h"
#include "subsystems/subsystems.hpp"
#include "subsystems/vision.hpp"

/**
*
* ROUTINE METHODS
*
*/

/**
* Standard match autonomous routine
*/
void matchAuto() {
	printf("Match auto\n");

	// STARTING POSITION
	arms::odom::reset({11.25, 50.25}, 271.0);

	// FLYWHEEL INIT
	flywheel::start(167);
	// DISC COUNTER INIT
	discCounter::setNum(1);
	// DEFLECTOR INIT
	deflector::down();

	// Get second disc
	printf("Getting disc 2\n");
	intake::toggle(600);
	arms::chassis::move({11.25, 19}); // move to disc 2

	// Get disc 3
	printf("Getting disc 3\n");
	arms::chassis::move({13.2074, 32.3512}, arms::REVERSE); // move back
	arms::chassis::turn(315);                               // turn to disc 3
	arms::chassis::move({16, 27});                          // pick up disc 3

	// Get roller
	printf("Getting roller\n");
	arms::chassis::move({12.5, 27.3512}); // move back
	arms::chassis::turn(5);               // turn to roller
	arms::chassis::tank(-25, -25); // braindead back into roller to apply pressure
	pros::delay(1000);             // get up to roller
	roller::move(100);             // turn roller
	pros::delay(70);
	arms::chassis::tank(0, 0);          // stop chassis
	roller::move(0);                    // stop roller mech
	turret::goto_angle(-61, 100, true); // for shot 1
	discLift::discLiftUp();             // for shot 1

	// Shoot 1st shot
	printf("Going to first shot\n");
	arms::chassis::move({13, 28});
	arms::chassis::turn(50, arms::THRU);
	intake::stop();
	arms::chassis::move({22, 49, 45});
	// vision::set_vision_offset(205); // aim offset for long distance shot
	turret::enable_vision_aim();
	flywheel::fire(3, 8000);
	turret::disable_vision_aim();
	turret::goto_angle(0, 100, true);

	// Aim turret, setup flywheel for next shot
	flywheel::set_target_speed(160);

	// Drive through next 3 discs
	printf("Driving through discs\n");
	intake::toggle(600);
	arms::chassis::move({55, 82}, 45);
	arms::chassis::waitUntilFinished(1);
	pros::delay(800);
	discLift::discLiftUp();
	turret::goto_angle(41, 100, true);

	// Shoot 2nd shot
	printf("Shooting second shot\n");
	arms::chassis::turn(290);
	pros::delay(1500);
	// vision::set_vision_offset(220);
	// turret::enable_vision_aim();
	flywheel::fire(3, 8000);
	flywheel::stop();
	turret::disable_vision_aim();
	turret::goto_angle(0, 100, true);
	flywheel::set_target_speed(0);
}

void discRushAuto() {
	using namespace arms::chassis;
	arms::odom::reset({35.5, 17}, 90);
	setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);

	// first 3 stack
	flywheel::start(140);
	discCounter::setNum(0);
	deflector::down();
	intake::start(5);
	turret::goto_angle(45, 200, true);
	move({35.5, 21}, 100, 1, arms::THRU);
	intake::start(-100);
	pros::delay(50);
	intake::start(100);
	turret::goto_angle(9, 200, true);
	pros::delay(500);
	move({35.5, 19}, 100, 1, arms::REVERSE);
	move({35.5, 24}, 100, 2);
	move({35.5, 12}, arms::REVERSE);
	turret::enable_vision_aim();
	pros::delay(100);
	flywheel::fire(3, 4000);
	turret::disable_vision_aim();
	
	// line disks, roller
	turret::goto_angle(0, 200, true);
	flywheel::start(140);
	intake::start(100);
	move({29, 16}, 70);
	turn(120);
	roller::move(100);
	tank(-40,-40);
	pros::delay(800);
	roller::move(0);
	tank(0,0);
	move({21.5,10, 185});


	// preload, shoot
	move({33,12}, arms::REVERSE);
	turn(0);
	turret::goto_angle(30, 200, true);
	intake::start(100);
	move({50,9});
	move({55,19});
	turret::enable_vision_aim();
	pros::delay(500);
	flywheel::fire(3, 4000);
	turret::disable_vision_aim();

	// second 3 stack
	flywheel::start(135);
	intake::start(100);
	move({60,33}, 40, 1, arms::THRU);
	turret::goto_global(120, 200);
	pros::delay(1000);
	turret::enable_vision_aim();
	flywheel::fire(3, 4000);
	turret::disable_vision_aim();
	
	// other line disks
	flywheel::start(130);
	intake::start(100);
	move({63,46}, 50, 2);
	move({59,41}, arms::REVERSE);
	turret::goto_angle(-45, 200, true);
	move({56,41}, 50, 2);
	move({59,41}, arms::REVERSE);
	turret::enable_vision_aim();
	pros::delay(500);
	flywheel::fire(3,4000);
	turret::disable_vision_aim();

	// low goal disks
	flywheel::start(145);
	move({87,12}, arms::REVERSE);
	turn(65);
	intake::start(100);
	turret::goto_angle(43, 200, true);
	move({91,32}, 40);
	turret::enable_vision_aim();
	pros::delay(1000);
	flywheel::fire(3, 4000);
	turret::disable_vision_aim();
}

/**
* Subsystem test routine
*/
void subsystem_test() {
	printf("Turret going to angle\n");
	turret::goto_angle(45);

	printf("Intaking 3 discs:\n");
	intake::start(300);
	int numFound = discCounter::expect(3, 10000);
	intake::stop();
	printf("    %d discs found\n", numFound);

	printf("Firing 3 discs:\n");
	flywheel::start(150);
	discLift::discLiftHold();
	pros::delay(500);
	int numFired = flywheel::fire(3, 10000);
	flywheel::stop();
	printf("    %d discs fired\n", numFired);
	discLift::discLiftDown();
}

/**
* Skils autonomous routine
*/
void skillsAuto() {
	vision::set_targ_goal(vision::Goal::BOTH);
	using namespace arms::chassis;
	arms::odom::reset({34,11},45);

	deflector::up();
	// spin roller
	flywheel::start(105);
	turret::goto_angle(-62, 200, true);
	roller::move(-100);
	tank(-20,-20);
	pros::delay(800);
	intake::start(-100);
	pros::delay(200);
	roller::move(0);
	tank(0,0);

	// first 3 stack
	intake::start(100);
	//flywheel::start(132);
	move({49,24}, 70, arms::THRU);
	move({66,37}, 30);
	turn(40, 100, 2);
	//turret::enable_vision_aim();
	pros::delay(500);
	flywheel::fire(3, 5000);
	//turret::disable_vision_aim();

	// 3 disks
	flywheel::start(135);
	deflector::down();
	turret::goto_angle(67, 200, true);
	intake::start(100);
	move({112,79}, 70);
	turn(90, 100, 2);
	//turret::enable_vision_aim();
	pros::delay(500);
	//move({112,88});
	flywheel::fire(3, 5000);
	//turret::disable_vision_aim();

	// second 3 stack
	//flywheel::start(120);
	deflector::up();
	flywheel::start(115);
	turret::goto_angle(-20, 200, true);
	intake::start(100);
	move({109,107}, 40);
	pros::delay(500);
	turn(180);
	move({102, 106});
	turn(185, 100, 2);
	flywheel::fire(3, 5000);
	//turret::disable_vision_aim();

	// third 3 stack
	intake::start(100);
	flywheel::start(105);
	turret::goto_angle(-32, 200, true);
	move({83,105},40);
	//turret::enable_vision_aim();
	turn(195, 100, 2);
	pros::delay(500);
	flywheel::fire(3, 5000);

	// 3 more disks
	flywheel::start(137);
	deflector::down();
	//turret::disable_vision_aim();
	turret::goto_angle(60, 200, true);
	intake::start(100);
	turn(220);
	move({32,62}, 70);
	turn(270, 100, 2);
	//turret::enable_vision_aim();
	flywheel::fire(3, 5000);
	//turret::disable_vision_aim();

	// fourth 3 stack
	//flywheel::start(120);
	intake::start(100);
	turret::goto_angle(-60, 200, true);
	move({31,35},40);
	turn(40, 100, 2);
	//turret::enable_vision_aim();
	flywheel::fire(3,5000);
	//turret::disable_vision_aim();

	// 2 disks
	intake::start(100);
	turret::goto_angle(-80, 200, true);
	move({59,59}, 70);
	turn(35, 100, 2);
	flywheel::fire(2, 4000);

	// 2 more disks
	intake::start(100);
	turret::goto_angle(65, 200, true);
	move({101,87}, 70);
	turn(220, 100, 2);
	flywheel::fire(2, 4000);

	turret::goto_angle(0, 200, true);
	move({132,111}, arms::REVERSE);
	endgame::deploy();
}

void rollerOnly() {
	roller::move(65);
	arms::chassis::tank(-20,-20);
	pros::delay(300);
	intake::start(-100);
	pros::delay(100);
	roller::move(0);
	arms::chassis::tank(0,0);
}

/**
*
* MAIN AUTONOMOUS METHOD
*
*/

/**
* Autonomous method--calls a method routine. 
* Implements autonomous selector (if competition is connected).
*/
void autonomous() {
	int auton = lcd2::selector::get_auton();
	if (auton > 0) {
		vision::set_targ_goal(vision::Goal::RED);
	} else if (auton < 0) {
		vision::set_targ_goal(vision::Goal::BLUE);
	}
	if(pros::competition::is_connected()){
		//Switch based on auton selector for matches
		switch (auton) {
			case 0:
				skillsAuto();
				break;
			case 3:
			case -3:
				rollerOnly();
				break;
			default:
				discRushAuto();
				break;
		}
	} else{
		//PLACE DESIRED AUTON FOR TUNING HERE: (will run when competion not connected)
		// vision::set_targ_goal(vision::Goal::BLUE);
		skillsAuto();
	}
	arms::chassis::setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
}