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
	turret::goto_angle(9, 200, true);
	move({35.5, 20.7}, 100, 1, arms::THRU);
	intake::start(-100);
	pros::delay(50);
	intake::start(100);
	pros::delay(500);
	move({35.5, 19}, 100, 1, arms::REVERSE);
	move({35.5, 24}, 100, 1);
	move({35.5, 12}, arms::REVERSE);
	turn(90);
	flywheel::fire(3, 4000);
	
	// line disks, roller
	flywheel::start(140);
	turn(142);
	intake::start(100);
	move({29, 16});
	turn(125);
	roller::move(65);
	tank(-40,-40);
	pros::delay(800);
	roller::move(0);
	tank(0,0);
	move({21.5,10, 180});


	// preload, shoot
	move({33,12}, arms::REVERSE);
	turn(0);
	intake::start(100);
	turret::goto_angle(49, 200, true);
	move({50,9}, arms::THRU);
	move({55,22});
	turret::enable_vision_aim();
	pros::delay(500);
	flywheel::fire(3, 4000);

	// second 3 stack
	flywheel::start(140);
	intake::start(100);
	move({58,38}, 30, 1, arms::THRU);
	pros::delay(500);
	flywheel::fire(3, 4000);
	turret::disable_vision_aim();
	turret::goto_angle(-50, 200, true);
	
	// other line disks
	flywheel::start(140);
	intake::start(100);
	move({58,49}, 50, 1, arms::THRU);
	move({63,43}, arms::REVERSE);
	turn(170);
	turret::enable_vision_aim();
	move({55,44}, 50, 1, arms::THRU);
	move({63,43}, arms::REVERSE);
	flywheel::fire(3,4000);
	turret::disable_vision_aim();
	turret::goto_angle(-60, 200, true);

	// low goal disks
	flywheel::start(150);
	move({83,63}, arms::REVERSE);
	turn(280);
	intake::start(100);
	move({91,18});
	turn(180);
	turret::enable_vision_aim();
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
	using namespace arms;

	flywheel::start(140);

	odom::reset({126, 81}, 180);

	// FLYWHEEL INIT
	flywheel::start(167);
	// DISC COUNTER INIT
	discCounter::setNum(1);
	// DEFLECTOR INIT
	deflector::down();

	intake::start(100);
	chassis::move({106, 84}, 80);
	chassis::turn(223, 100);
	chassis::move({82, 58, 223}, 50);
	pros::delay(500);
	intake::stop();
	chassis::turn(315);

	flywheel::fire(3, 4000);

	chassis::turn(223);
	intake::start(100);
	chassis::move({60, 34, 223}, 60);
	pros::delay(500);
	chassis::move({34, 10}, 30);
	pros::delay(500);
	intake::stop();
	chassis::turn(0);

	flywheel::fire(3, 4000);

	intake::start(100);
	discLift::discLiftDown();
	chassis::turn(180);
	chassis::move({12, 10}, 30);
	pros::delay(1000);
	chassis::turn(45, 60);
	chassis::move({24, 24, 45}, 40);
	pros::delay(1000);
	intake::stop();
	pros::delay(1000);

	flywheel::fire(2, 4000);

	intake::start(100);
	discLift::discLiftDown();
	chassis::move(10, 60);
	pros::delay(500);
	chassis::move({36, 36}, 40);
	pros::delay(300);
	intake::start(-100);
	pros::delay(300);
	intake::stop();
	flywheel::fire(3, 4000);
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
		vision::set_targ_goal(vision::Goal::BLUE);
		discRushAuto();
	}
	arms::chassis::setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
}