#include "main.h"
#include "ARMS/api.h"
#include "subsystems/subsystems.hpp"

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
	// INTAKE INIT
	intake::toggle_arm();
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
	arms::odom::reset({35.5, 17}, 90);
	arms::chassis::setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);

	flywheel::start(131);
	discCounter::setNum(0);
	deflector::down();
	intake::start(100);
	turret::goto_angle(11, 100, true);
	arms::chassis::move({35.5, 21}, 50, arms::THRU);
	discCounter::expect(3, 2000);
	flywheel::fire(3, 3000);
	
	//arms::chassis::turn(191);
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
	// INTAKE INIT
	intake::toggle_arm();
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
	if(pros::competition::is_connected()){
		//Switch based on auton selector for matches
		switch (arms::selector::auton) {
			case 0:
				// skillsAuto();
				break;
			default:
				matchAuto();
				break;
		}
	} else{
		//PLACE DESIRED AUTON FOR TUNING HERE: (will run when competion not connected)
		discRushAuto();
	}
}