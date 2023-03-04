#include "ARMS/chassis.h"
#include "main.h"

#include "subsystems/disccounter.hpp"
#include "subsystems/flywheel.hpp"
#include "subsystems/intake.hpp"
#include "subsystems/roller.hpp"
#include "subsystems/turret.hpp"
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

 void fire3(){
	flywheel::start(200);
	flywheel::fire(3, 20000);
 }
void matchAuto(){
	printf("Match auto\n");

	//STARTING POSITION
	arms::odom::reset({11.25, 50.25}, 271.0); 

	//FLYWHEEL INIT
	flywheel::start(200);
	//DISC COUNTER INIT
	disccounter::setNum(1);
	
	//Get first disc
	printf("Getting first disc\n");
	intake::toggle(600);
	arms::chassis::move({11.25, 20}); //move to disc 1

	//Get roller
	printf("Getting roller, disc 2\n");
	arms::chassis::move({12.7074, 28.3512}, arms::REVERSE); //move to roller
	arms::chassis::turn(10); //turn to roller and pick up disc 2
	arms::chassis::tank(-18, -18); //braindead back into roller to apply pressure
	pros::delay(500); //get up to roller
	roller::move(100); //turn roller
	pros::delay(120);
	arms::chassis::tank(0, 0); //stop chassis
	roller::move(0); //stop roller mech
	
	//Shoot discs 1, 2, 3
	printf("Shooting discs 1, 2, 3\n");
	turret::goto_angle(12,100,true); //for shot 1
	turret::enable_vision_aim();
	arms::chassis::move({15.9607, 27.2107});
	arms::chassis::turn(0);
	flywheel::fire(3, 7000);
	turret::disable_vision_aim();

	//Aim turret for next shot
	turret::goto_angle(5,100,true); //for discs 4, 5, 6

	//Get disc 4
	printf("Getting disc 4\n");
	arms::chassis::move({16.1328, 24.2143}, 336.4); //move to disc 4

	#if 0
	//Get disc 5
	printf("Getting disc 5\n");
	arms::chassis::turn(65.5, arms::THRU); //turn to disc 5
	arms::chassis::move({28.7356, 51.7074}, arms::THRU); //move to disc 5

	//Get disc 6
	printf("Getting disc 6\n");
	arms::chassis::turn(353, arms::THRU); //turn to disc 6
	arms::chassis::move({36.7881, 50.344}); //move to disc 6

	//Shoot discs 4, 5, 6
	printf("Shooting discs 4, 5, 6\n");
	turret::enable_vision_aim();
	flywheel::fire(disccounter::disc_count(), 5000);
	turret::disable_vision_aim();

	//Aim turret for next shot
	turret::goto_angle(0,100,true); //for shot 7, 8, 9

	//Get disc 7, 8
	printf("Getting discs 7, 8\n");
	arms::chassis::move({23.4173, 49.509}, 50.6, arms::REVERSE | arms::THRU); //back up and aim at discs 7, 8
	arms::chassis::move({52.6244, 84.4596}, 49, arms::THRU); //drive thru 7 and 8

	//Get disc 9
	// printf("Getting disc 9\n");
	// arms::chassis::turn(-80, arms::THRU); //turn to disc 9

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
	disclift::discLiftHold();
	pros::delay(500);
	int numFired = flywheel::fire(3, 10000);
	flywheel::stop();
	printf("    %d discs fired\n", numFired);
	disclift::discLiftDown();
}

void autonomous() {
	// matchAuto();
	fire3();
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