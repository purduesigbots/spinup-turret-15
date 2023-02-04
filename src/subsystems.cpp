#include "main.h"
#include "math.h"
#include "pros/misc.hpp"
#include "pros/motors.h"
#include "subsystems.h"
#include <cmath>

// intake -------------------------------------------------------------------------
namespace intake {

Motor left_motor(11, MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_ROTATIONS);
Motor right_motor(19, MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_ROTATIONS);

double speed = 0;

void move(double speed) {
    left_motor.move_voltage(120 * speed);
    right_motor.move_voltage(120 * speed);
    intake::speed = speed;
}

} // intake

// roller -------------------------------------------------------------------------
namespace roller {

Motor motor(6, MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_ROTATIONS);
Optical optical(5);

double speed = 0;
bool turning_roller = false;
int roller_turning_speed = 80;
bool last_hue = false;

bool isRed() {
    double color = optical.get_hue();
    if (color > 180) {
        return (color - 360 > -30);
    } else {
        return color < 30;
    }
}

void toggle_turn_roller() {
    turning_roller = !turning_roller;
    last_hue = isRed();
}

void move(double speed) {
    roller::speed = speed;
}

void task() {
    while (true) {
        lcd::set_text(0, "Optical: " + std::to_string(optical.get_hue()));
        if (turning_roller) {
            motor.move(roller_turning_speed);
            if (last_hue ^ isRed()) {
                turning_roller = false;
                motor.move(-20);
            }
        } else {
            motor.move(120 * speed);
        }
        pros::delay(10);
    }
}

void init() {
    optical.set_led_pwm(100);
    pros::Task roller_task(task);
}

} // roller

// turret -------------------------------------------------------------------------
namespace turret {

Motor motor(7, MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_ROTATIONS);

double speed = 0;
double target_angle = 0;
double current_angle = 0;

void move(double speed) {
    motor.move_voltage(120 * speed);
    turret::speed = speed;
}

void task() {
    while (true) {
        current_angle = motor.get_position() * 0.01;
        motor.move_voltage(120 * (speed + (target_angle-current_angle) * 1));
    }
}

} // turret

namespace disklift {
    pros::Motor lift_motor(21, pros::E_MOTOR_GEARSET_36, true, pros::E_MOTOR_ENCODER_DEGREES);
    double lift_pos[] = {-15, 55, 68, 78}; //(DEPRECATED) -JBH 2/1/23
    int deltaDown = 2;
    int newPos = 0;
    int i = 0; // (DEPRECATED) -JBH 2/1/23
    double liftDownPos = -13;
    void move(double speed);
    void move_to(double position,double speed){
       lift_motor.move_absolute(position, speed);
    }
    void discLiftUp(){
        lift_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        // if(lift_motor.get_position() < 70){
        //     lift_motor.move_voltage(12000);
        // } else{
        //     newPos = 78;
        //     lift_motor.move_absolute(78,100);
        // }
        lift_motor.move_voltage(12000);
    }
    void calculatePos(){
        // if(lift_motor.get_position() > 72){
        //     //3 discs in mag
        //     newPos = lift_motor.get_position() - 5;
        // } else if(lift_motor.get_position() > 60){
        //     newPos = lift_motor.get_position() - 9;
        // } else{
        //     newPos = lift_motor.get_position() - 5;
        // }
        newPos = lift_motor.get_position();
    }
    void discLiftHold(){
        lift_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        //When called, hold disc lift in place (enough force to give indexer effective traction)
        // lift_motor.move_absolute(newPos,100);
        lift_motor.move_voltage(6000);
    }
    void discLiftDown(){
        lift_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        if(std::abs(liftDownPos - lift_motor.get_position()) < 3){
            // Acceptable tolerance, avoid burnout
            lift_motor.move_voltage(0);
        } else{
            lift_motor.move_absolute(liftDownPos,100);
        }
    }
}

namespace flywheel {
// flywheel tuning
int threshold = 50;
double kV = 65.4;
double kP = 0.5;

// auton flywheel tunings
int sy_threshold = 10;
double sy_kV = 68;
double sy_kP = 30;

sylib::SpeedControllerInfo motor_speed_controller (
    [](double rpm){return kV;}, // kV function - 120
    sy_kP, // kP - 1
    0, // kI
    0.25, // kD - 0.5
    0, // kH
    true, // anti-windup enabled
    3, // anti-windup range
    true, // p controller bounds threshold enabled
    3, // p controller bounds cutoff enabled - 5
    sy_kP/4, // kP2 for when over threshold - 0.25
    sy_threshold // range to target to apply max voltage - 10
);

// flywheel motors
// pros::Motor flywheel1(9);
// pros::Motor flywheel2(1, 1);
// pros::Motor_Group motor({flywheel1, flywheel2});
sylib::Motor left_flywheel(9, 200, false, motor_speed_controller);
sylib::Motor right_flywheel(10, 200, true, motor_speed_controller);
pros::Motor indexer (14, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
double speed = 0;

void move(double speed) {
    flywheel::speed = speed;
}

#define SMA_LEN 5

static double sma_data[SMA_LEN];
static int count=0;
static double average;

/* void add_data() {
    double reading = flywheel1.get_actual_velocity();
    average -= sma_data[count]/SMA_LEN;
    average += reading/SMA_LEN;
    sma_data[count++] = reading;
    count = count < SMA_LEN ? count : 0;
} */

bool at_speed() {
    return speed - average < 4;
}

void wait_until_fired() {
    while (at_speed()) {
        pros::delay(10);
    }
}

void wait_until_at_speed() {
    while (!at_speed()) {
        pros::delay(10);
    }
}

void task() {    
    uint32_t clock = sylib::millis();

    left_flywheel.set_braking_mode(kV5MotorBrakeModeCoast);
    right_flywheel.set_braking_mode(kV5MotorBrakeModeCoast);

    bool stopped = false;

    while(1) {

        sylib::delay_until(&clock,10);


        average = left_flywheel.get_velocity();
        //printf("%.2f,%d\n",average, flywheel1.get_applied_voltage()/120);

        if(speed == 0) {
            left_flywheel.stop();
            right_flywheel.stop();
            continue;
        }

        
       
        // if autonomous, use sylib controller
        left_flywheel.set_velocity_custom_controller(speed);
        right_flywheel.set_velocity_custom_controller(speed);
    }


    
}

void fire(){
    indexer.move_voltage(12000);
}

void stopIndexer(){
    indexer.move_voltage(0);
}

}
