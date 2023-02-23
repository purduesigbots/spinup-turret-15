#include "flywheel.hpp"

#include "api.h"
#include "main.h"
#include "subsystems/subsystems.hpp"

using namespace pros;

namespace flywheel {
    
// flywheel tuning
int threshold = 150;
double kV = 57.7;
double kP = 0.5;
double kI = 0.001;
double kD = 0.37;

// SILVA : GOLDY
int leftPort = isSilva() ? 9 : 9;
int rightPort = isSilva() ? 8 : 10;

sylib::SpeedControllerInfo motor_speed_controller (
    [](double rpm){return kV;}, // kV function - 120
    kP, // kP - 1
    kI, // kI
    kD, // kD - 0.5
    0, // kH
    true, // anti-windup enabled
    36, // anti-windup range
    false, // p controller bounds threshold enabled
    3, // p controller bounds cutoff enabled - 5
    kP/4, // kP2 for when over threshold - 0.25
    threshold // range to target to apply max voltage - 10
);

sylib::Motor left_flywheel(leftPort, 200, false, motor_speed_controller);
sylib::Motor right_flywheel(rightPort, 200, true, motor_speed_controller);  

pros::Motor indexer (14, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
double targetSpeed = 0;

#define SMA_LEN 5

const double STOP = 0.0;

static double sma_data[SMA_LEN];
static int count=0;
static double average_speed;

void task_function(void* data) {
    uint32_t clock = sylib::millis();

    left_flywheel.set_braking_mode(kV5MotorBrakeModeCoast);
    right_flywheel.set_braking_mode(kV5MotorBrakeModeCoast);

    bool stopped = false;

    while(1) {
        // Is there a reason that we are using sylib's delay instead of PROS?
        sylib::delay_until(&clock,10);
        average_speed = left_flywheel.get_velocity();

        // printf("%.2f,%.2f,%.2f\n", 
        //     average_speed, 
        //     targetSpeed, 
        //     left_flywheel.get_applied_voltage()/80.0
        // );

        // If the speed is less than or equal to zero, we want to stop the
        // motors and let them coast. Otherwise, we want them to adjust to the
        // proper speed
        if(targetSpeed == STOP) {
            left_flywheel.stop();
            right_flywheel.stop();
        }
        else {
            // if autonomous, use sylib controller
            // ^^^ This comment makes no sense. What here determines that we
            //     are in autonomous???
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

    if(flywheel::targetSpeed <= STOP) {
        flywheel::targetSpeed = STOP;
    }
}

void stop() {
    targetSpeed = STOP;
}

void toggle(double targetSpeed) {
    if(targetSpeed == STOP) {
        start(targetSpeed);
    }
    else {
        stop();
    }
}

bool at_speed() {
    // Check that the turret's RPM is within 3% of the target speed. 
    return std::abs(targetSpeed - average_speed) / targetSpeed < 0.03;
}


double curret_speed() {
    return average_speed;
}

double target_speed() {
    return targetSpeed;
}

// Blocks until the robot reaches a specific speed.
void wait_until_at_speed(uint32_t timeout) {
    while (!at_speed()) {
        printf("wait_until_at_speed\n");
        pros::delay(10);
    }
}

void wait_until_fired() {
    while (targetSpeed - average_speed < 20) {
        printf("wait_until_fired\n");
        pros::delay(10);
    }
}

int fire(int numDiscs, int timeout) {
    uint32_t startTime = pros::millis();

    int numberFired = 0;

    while(pros::millis() - startTime < timeout) {
        while(!at_speed()) {
            if(pros::millis() - startTime < timeout) {
                return numberFired;
            }
            pros::delay(10);
        }

        indexer.move_voltage(12000);
        numberFired++;
        disccounter::decrement();

        pros::delay(10);
    }

    return numberFired;
}

} // namespace flywheel