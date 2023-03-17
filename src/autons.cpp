#include "ARMS/chassis.h"
#include "ARMS/flags.h"
#include "main.h"

#include "ARMS/api.h"
#include "subsystems/deflector.hpp"
#include "subsystems/discLift.hpp"
#include "subsystems/disccounter.hpp"
#include "subsystems/flywheel.hpp"
#include "subsystems/intake.hpp"
#include "subsystems/roller.hpp"
#include "subsystems/turret.hpp"
#include "vision.h"

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

void fire3() {
	flywheel::start(200);
	flywheel::fire(3, 8000);
}

void matchAuto() {
	printf("Match auto\n");

	// STARTING POSITION
	arms::odom::reset({11.25, 50.25}, 271.0);

	// FLYWHEEL INIT
	flywheel::start(167);
	// DISC COUNTER INIT
	disccounter::setNum(1);
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
	disclift::discLiftUp();             // for shot 1

	// Shoot 1st shot
	printf("Going to first shot\n");
	arms::chassis::move({13, 28});
	arms::chassis::turn(50, arms::THRU);
	intake::stop();
	arms::chassis::move({22, 49, 45});
	vision::set_vision_offset(205); // aim offset for long distance shot
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
	vision::set_vision_offset(220);
	// turret::enable_vision_aim();
	flywheel::fire(3, 8000);
	flywheel::stop();
	turret::disable_vision_aim();
	turret::goto_angle(0, 100, true);
	flywheel::set_target_speed(0);
#if 0
	flywheel::set_target_speed(160);

	//get disc 7
	intake::toggle(600);
	arms::chassis::move({55, 72}, arms::THRU);
	arms::chassis::move({55, 82}, arms::REVERSE);

	//get disc 8
	arms::chassis::turn(180, arms::THRU); //turn to face disc 8
	arms::chassis::move({49, 65, 270}, arms::THRU);
	turret::goto_angle(20, 100, true); //turn turret for next shot
	arms::chassis::turn(315); //turn through disc, pick it up
	arms::chassis::waitUntilFinished(1);
	turret::enable_vision_aim();
	pros::delay(800); //wait until disc picked all the way up

	//take next shot
	flywheel::fire(2, 6000);
	turret::disable_vision_aim();
	turret::goto_angle(0, 100, true);
	
	//setup for next discs
	intake::start(600);
	arms::chassis::turn(90, arms::THRU);
	arms::chassis::move({43, 76});
	arms::chassis::move({43, 70}, arms::REVERSE);
	arms::chassis::move({35, 76, 135});
	arms::chassis::move({35, 70, 90}, 40, arms::REVERSE);
	arms::chassis::move({27, 76, 135});
	arms::chassis::waitUntilFinished(1);
	pros::delay(800);
	turret::goto_angle(0, 100, true);
	arms::chassis::move({48, 72}, arms::REVERSE);
	arms::chassis::turn(327);
	turret::enable_vision_aim();
	flywheel::fire(3, 9000);
	turret::disable_vision_aim();
#endif
}
extern "C" {

void subsystem_test() {
	printf("Turret going to angle\n");
	turret::goto_angle(45);

	printf("Intaking 3 discs:\n");
	intake::start(300);
	int numFound = disccounter::expect(3, 10000);
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

void autonomous() {
	skillsAuto();
	// fire3();
	// vision::set_vision_offset(true);
	// roller::set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

	// switch (arms::selector::auton) {
	// 	case 0:
	// 		// skillsAuto();
	// 		break;
	// 	default:
	// 		matchAuto();
	// 		break;
	// }
	// turret::goto_angle(0,250,true);
}
}