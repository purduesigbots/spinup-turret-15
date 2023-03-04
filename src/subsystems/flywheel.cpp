#include "flywheel.hpp"

#include <stdint.h>

#include "api.h"
#include "disccounter.hpp"
#include "disclift.hpp"
#include "main.h"
#include "subsystems/subsystems.hpp"

#include "ARMS/config.h"

using namespace pros;

namespace flywheel {

sylib::SpeedControllerInfo
    auton_speed_controller([](double rpm) { return 120; }, // kV function - 120
                           16,                             // kP - 1
                           0.001,                          // kI
                           0,                              // kD - 0.5
                           0,                              // kH
                           true,  // anti-windup enabled
                           36,    // anti-windup range
                           false, // p controller bounds threshold enabled
                           3,     // p controller bounds cutoff enabled - 5
                           4,     // kP2 for when over threshold - 0.25
                           50     // range to target to apply max voltage - 10
    );

// flywheel tuning
int threshold = FLYWHEEL_THRESHOLD;
double kV = FLYWHEEL_KV;
double kP = FLYWHEEL_KP;
double kI = FLYWHEEL_KI;
double kD = FLYWHEEL_KD;

sylib::SpeedControllerInfo motor_speed_controller(
    [](double rpm) { return kV; }, // kV function - 120
    kP,                            // kP - 1
    kI,                            // kI
    kD,                            // kD - 0.5
    0,                             // kH
    true,                          // anti-windup enabled
    36,                            // anti-windup range
    false,                         // p controller bounds threshold enabled
    3,                             // p controller bounds cutoff enabled - 5
    kP / 4,                        // kP2 for when over threshold - 0.25
    threshold                      // range to target to apply max voltage - 10
);

sylib::Motor left_flywheel(FLYWHEEL_LEFT, 200, false, motor_speed_controller);
sylib::Motor right_flywheel(FLYWHEEL_RIGHT, 200, true, motor_speed_controller);

pros::Motor indexer(INDEXER_PORT, pros::E_MOTOR_GEARSET_18, false,
                    pros::E_MOTOR_ENCODER_DEGREES);
double targetSpeed = 0;

#define SMA_LEN 5

const double STOP = 0.0;

static double sma_data[SMA_LEN];
static int count = 0;
static double average_speed;

void task_function(void* data) {
	uint32_t clock = sylib::millis();

	left_flywheel.set_braking_mode(kV5MotorBrakeModeCoast);
	right_flywheel.set_braking_mode(kV5MotorBrakeModeCoast);

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
			right_flywheel.stop();
		} else {
			left_flywheel.set_velocity_custom_controller(targetSpeed);
			right_flywheel.set_velocity_custom_controller(targetSpeed);
		}
	}
}

void initialize() {
	printf("Initializing flywheel subsystems: ");
	Task task(task_function);
	printf("Done\n");
}

// Starts the flywheel and tells it to target a specific speed
void start(double targetSpeed) {
	flywheel::targetSpeed = targetSpeed;
}

// Tells the flywheel to target a certain speed.
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

double current_speed(int n) {
	if (n == 1) {
		return left_flywheel.get_velocity();
	} else if (n == 2) {
		return right_flywheel.get_velocity();
	} else {
		return 0;
	}
}

double target_speed() {
	return targetSpeed;
}

// Blocks until the robot reaches a specific speed.
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
	uint32_t startTime = pros::millis();
	uint32_t endTime = startTime + timeout;

	int numberFired = 0;
    intake::start(600);
    disclift::discLiftUp();
    pros::delay(460);
    intake::stop();
	// While we haven't fired all the discs we want to fire
	while (numberFired < numDiscs) {
        disclift::discLiftUp();
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
		disclift::discLiftHold();

		// We wait until we detect that the disc is fired or that the timeout is
		// reach (if timeouts are enabled.). See the note above
		if (wait_until_fired(timeLeft) && timeout > 0) {
			goto RETURN;
		}

		// Now that the flywheel is at speed, we start the indexer
		indexer.move_voltage(0);
		// Update the disc counter and number of discs we've fired
		numberFired++;
		disccounter::decrement();
		pros::delay(numberFired * 400);
		targetSpeed++; //increase to help recovery...???? could be gas?????
	}
    RETURN:
    intake::stop();
	disclift::discLiftDown();
	indexer.move_voltage(0);
	return numberFired;
}

void fireControl_driver(bool enable) {
	indexer.move_voltage(12000 *
	                     enable); // sets to 12000 if enable is true, 0 if false
}

void debug_screen() {
	pros::lcd::print(0, "Flywheel Info:");
	pros::lcd::print(1, " Cur Speed: %f", current_speed());
	pros::lcd::print(2, " Tgt Speed: %f", target_speed());
	pros::lcd::print(4, " At Speed?: %d", at_speed());
    pros::lcd::print(5, "Left temp: %3.2f", left_flywheel.get_temperature());
    pros::lcd::print(6, "Right temp: %3.2f", right_flywheel.get_temperature());
}

} // namespace flywheel