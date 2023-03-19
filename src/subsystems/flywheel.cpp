#include "main.h"
#include "ARMS/config_silver.h"
#include "subsystems/subsystems.hpp"

using namespace pros;

namespace flywheel {

	namespace{ //Anonymous namespace for private data and methods

		/**
		*
		* PRIVATE DATA
		*
		*/

		//SYLIB flywheel speed controller
		sylib::SpeedControllerInfo motor_speed_controller(
			// kV function - returns kV constant, sylib handles multiplying by rpm 
			[](double rpm) { return FLYWHEEL_KV; }, 
			FLYWHEEL_KP,					// kP
			FLYWHEEL_KI,					// kI
			FLYWHEEL_KD,					// kD
			FLYWHEEL_KH,					// kH
			false,                          // anti-windup enabled
			36,                            	// anti-windup range
			false,                         	// p controller bounds threshold enabled
			3,                             	// p controller bounds cutoff enabled - 5
			0,                        		// kP2 for when over threshold - 0.25
			FLYWHEEL_THRESHOLD              // range to target to apply max voltage - 10
		);

		//SYLIB motor object for left flywheel motor
		sylib::Motor left_flywheel(FLYWHEEL_LEFT, 200, false, motor_speed_controller);
		//PROS motor object f or right flywheel motor; will be slaved to SYLIB's calculation for the left motor
		pros::Motor right_flywheel(FLYWHEEL_RIGHT, pros::E_MOTOR_GEARSET_18, true);

		//The motor used to control the indexer
		pros::Motor indexer(INDEXER_PORT, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
		
		//Local variable for target speed
		double targetSpeed = 0;

		//Local const for stopped speed
		const double STOP = 0.0;

		//Local static variable for average speed--current one motor master/slave system overrides this 
		//with the left motor's speed. I am leaving it here anyways in case we want to return to a two
		//motor system in the future. - JBH 3/17/23
		static double average_speed;

		/**
		*
		* PRIVATE METHODS
		*
		*/

		/**
		* The task that controls the flywheel.
		*/
		void task_function(void* data) {
			uint32_t clock = sylib::millis();

			left_flywheel.set_braking_mode(kV5MotorBrakeModeCoast);
			right_flywheel.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

			bool stopped = false;

			while (1) {
				// Is there a reason that we are using sylib's delay instead of PROS?
				sylib::delay_until(&clock, 10);
				average_speed = left_flywheel.get_velocity();

				// printf("%.2f,%.2f,%.2f\n",
				//     average_speed,
				//     targetSpeed,
				//     left_flywheel.get_applied_voltage()/80.0
				// );

				// If the speed is less than or equal to zero, we want to stop the
				// motors and let them coast. Otherwise, we want them to adjust to the
				// proper speed
				if (targetSpeed == STOP) {
					left_flywheel.stop();
					right_flywheel = 0;
				} else {
					
					left_flywheel.set_velocity_custom_controller(targetSpeed);
					right_flywheel.move_voltage(left_flywheel.get_applied_voltage());
				}
			}
		}
	}	
	
	/**
	*
	* PUBLIC METHODS (see header file for documentation)
	*
	*/

	void initialize() {
		printf("Initializing flywheel subsystems: ");
		Task task(task_function);
		printf("Done\n");
	}

	void start(double targetSpeed) {
		flywheel::targetSpeed = targetSpeed;
	}

	void set_target_speed(double targetSpeed) {
		flywheel::targetSpeed = targetSpeed;
	}

	void change_target_speed(double amount) {
		flywheel::targetSpeed += amount;

		if (flywheel::targetSpeed <= STOP) {
			flywheel::targetSpeed = STOP;
		}
	}

	void stop() {
		targetSpeed = STOP;
	}

	void toggle(double targetSpeed) {
		if (targetSpeed != STOP && targetSpeed != flywheel::targetSpeed) {
			start(targetSpeed);
		} else {
			stop();
		}
	}

	bool at_speed() {
		// Check that the turret's RPM is within 3% of the target speed.
		return std::abs(targetSpeed - average_speed) / targetSpeed < 0.20;
	}

	double current_speed() {
		return average_speed;
	}

	double target_speed() {
		return targetSpeed;
	}

	bool wait_until_at_speed(uint32_t timeout) {
		uint32_t startTime = pros::millis();
		while (!at_speed()) {
			// timeout check
			if (timeout > 0 && pros::millis() - startTime >= timeout) {
				return true;
			}
			pros::delay(10);
		}
		pros::delay(1000); //do it a second time to make sure it's settled in 
		while (!at_speed()) {
			// timeout check
			if (timeout > 0 && pros::millis() - startTime >= timeout) {
				return true;
			}
			pros::delay(10);
		}
		return false;
	}

	bool wait_until_fired(uint32_t timeout) {
		uint32_t startTime = pros::millis();
		while (targetSpeed - average_speed < 20) {
			// timeout check
			if (timeout > 0 && pros::millis() - startTime >= timeout) {
				return true;
			}
			pros::delay(10);
		}
		return false;
	}

	int fire(int numDiscs, int timeout) {
		//Init local business
		uint32_t startTime = pros::millis();
		uint32_t endTime = startTime + timeout;
		int numberFired = 0;
		intake::start(600);
		discLift::discLiftUp();
		pros::delay(460);
		intake::stop();

		// While we haven't fired all the discs we want to fire
		while (numberFired < numDiscs) {
			discLift::discLiftUp();
			// Check if we've reached the timeout and return if we have
			uint32_t timeLeft = endTime - pros::millis();
			if (timeLeft <= 0 && timeout > 0) {
				goto RETURN;
			}
			// We first wait to make sure that the flywheel is at the speed we want.
			// This ensures the flywheel shoots consistantly. If timeouts are
			// enabled, and we reach the timeout before we are at speed, we return.
			//
			// NOTE: It is important that the call to wait_until_at_speed() is first
			//       in these if statements due to short circuit evaluation. If the
			//       "timeout > 0" is first, the entire if will fail
			//       wait_until_at_speed() will never run
			if (wait_until_at_speed(timeLeft) && timeout > 0) {
				goto RETURN;
			}

			// Now that the flywheel is at speed, we start the indexer
			indexer.move_voltage(12000);
			discLift::discLiftHold();

			// We wait until we detect that the disc is fired or that the timeout is
			// reach (if timeouts are enabled.). See the note above
			if (wait_until_fired(timeLeft) && timeout > 0) {
				goto RETURN;
			}

			// Now that the flywheel is at speed, we start the indexer
			indexer.move_voltage(0);
			// Update the disc counter and number of discs we've fired
			numberFired++;
			discCounter::decrement();
			pros::delay(numberFired * 400);

			//Increment target speed by 1 on the 200rpm scale for each disc fired, janky way 
			//to make sure the flywheel doesn't slow down too much. 
			//DEPRECATED IN FAVOR OF BETTER FLYWHEEL TUNING - JBH 3/17/23
			// targetSpeed += numberFired; 
		}
		RETURN: // GOTO thingy for ending the function w/o duplicate code everywhere
		intake::stop();
		discLift::discLiftDown();
		indexer.move_voltage(0);
		return numberFired;
	}

	void fireControl_driver(bool enable) {
		indexer.move_voltage(12000 * enable); // sets to 12000 if enable is true, 0 if false
	}

	void debug_screen() {
		pros::lcd::print(0, "Flywheel Info:");
		pros::lcd::print(1, " Cur Speed: %3.2f, Tgt: %3.2f", current_speed(), target_speed());
		pros::lcd::print(2, " At Speed?: %s", at_speed()? "True" : "False");
		pros::lcd::print(3, " Left temp: %3.2f", left_flywheel.get_temperature());
		pros::lcd::print(4, " Right temp: %3.2f", right_flywheel.get_temperature());
		pros::lcd::print(5, " (Applied) R: %5d L: %5d", right_flywheel.get_voltage(), left_flywheel.get_applied_voltage());
	}
}