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
	pros::delay(500);
	turret::goto_angle(9, 200, true);
	move({35.5, 19}, 100, 1, arms::REVERSE);
	move({35.5, 24}, 100, 2);
	move({35.5, 12}, arms::REVERSE);
	turret::enable_vision_aim();
	pros::delay(100);
	flywheel::fire(3, 4000);
	turret::disable_vision_aim();
	
	// line disks, roller
	turret::goto_angle(0, 200, true);
	flywheel::start(137);
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
	intake::start(100);
	move({50,9});
	move({55,21});
	turret::goto_global(112, 200);
	pros::delay(500);
	turret::enable_vision_aim();
	flywheel::fire(3, 4000);
	turret::disable_vision_aim();

	// second 3 stack
	flywheel::start(135);
	intake::start(100);
	move({58,38}, 40, 1, arms::THRU);
	turret::goto_global(120, 200);
	pros::delay(1000);
	turret::enable_vision_aim();
	flywheel::fire(3, 4000);
	turret::disable_vision_aim();
	
	// other line disks
	flywheel::start(130);
	intake::start(100);
	move({57,50}, 50, 2);
	move({59,41}, arms::REVERSE);
	move({52,43}, 50, 2);
	turret::goto_global(145, 200);
	move({59,41}, arms::REVERSE);
	turret::enable_vision_aim();
	pros::delay(500);
	flywheel::fire(3,4000);
	turret::disable_vision_aim();

	// low goal disks
	flywheel::start(140);
	move({81,16}, arms::REVERSE);
	turn(65);
	intake::start(100);
	turret::goto_angle(43, 200, true);
	move({86,45}, 40);
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
	flywheel::start(110);
	turret::goto_angle(-62, 200, true);
	roller::move(-100);
	tank(-20,-20);
	pros::delay(600);
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
	turret::enable_vision_aim();
	pros::delay(500);
	flywheel::fire(3, 5000);
	turret::disable_vision_aim();

	// 3 disks
	flywheel::start(132);
	deflector::down();
	turret::goto_angle(70, 200, true);
	intake::start(100);
	move({112,79}, 70);
	turn(90, 100, 2);
	turret::enable_vision_aim();
	pros::delay(500);
	//move({112,88});
	flywheel::fire(3, 5000);
	turret::disable_vision_aim();

	// second 3 stack
	//flywheel::start(120);
	deflector::up();
	flywheel::start(115);
	turret::goto_angle(-27, 200, true);
	intake::start(100);
	move({107,110}, 40);
	pros::delay(500);
	turn(180);
	turret::enable_vision_aim();
	move({99, 106});
	turn(197, 100, 2);
	flywheel::fire(3, 5000);
	turret::disable_vision_aim();

	// third 3 stack
	intake::start(100);
	turret::goto_angle(-32, 200, true);
	move({80,101},40);
	turret::enable_vision_aim();
	turn(195, 100, 2);
	pros::delay(500);
	flywheel::fire(3, 5000);

	// 3 more disks
	flywheel::start(137);
	deflector::down();
	turret::disable_vision_aim();
	turret::goto_angle(70, 200, true);
	intake::start(100);
	turn(225);
	move({34,51}, 70);
	turn(270, 100, 2);
	turret::enable_vision_aim();
	flywheel::fire(3, 5000);
	turret::disable_vision_aim();

	// fourth 3 stack
	//flywheel::start(120);
	intake::start(100);
	turret::goto_angle(-55, 200, true);
	move({38,25},40);
	turn(45, 100, 2);
	turret::enable_vision_aim();
	flywheel::fire(3,5000);
	turret::disable_vision_aim();

	// 2 disks
	intake::start(100);
	turret::goto_angle(-80, 200, true);
	move({58,56}, 70);
	turn(45, 100, 2);
	flywheel::fire(2, 4000);
	return;

	// 2 more disks
	intake::start(100);
	turret::goto_angle(70, 200, true);
	move({84,105}, 70);
	turn(245, 100, 2);
	flywheel::fire(2, 4000);

	turret::goto_angle(0, 200, true);
	move({93,137}, arms::REVERSE);
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
			default:
				discRushAuto();
				break;
		}
	} else{
		//PLACE DESIRED AUTON FOR TUNING HERE: (will run when competion not connected)
		// vision::set_targ_goal(vision::Goal::BLUE);
		discRushAuto();
	}
	arms::chassis::setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
}