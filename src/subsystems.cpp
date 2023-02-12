#include "main.h"
#include "math.h"
#include "pros/adi.hpp"
#include "pros/misc.hpp"
#include "pros/motors.h"
#include "subsystems.h"
#include <cmath>

// intake -------------------------------------------------------------------------
namespace intake {

int smart_port = 8;
char adi_port = 'a';
ADIDigitalOut intake_piston({{smart_port,adi_port}});
Motor left_motor(11, MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_ROTATIONS);
Motor right_motor(19, MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_ROTATIONS);
ADIAnalogIn line(6);

bool state = false;
double speed = 0;

void move(double speed) {
    left_motor.move_voltage(120 * speed);
    right_motor.move_voltage(120 * speed);
    intake::speed = speed;
}

void toggle(){
    state = !state;
    intake_piston.set_value(state);
}

bool clear() {
    return line.get_value() > 1500;
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

Motor motor(7, MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_ROTATIONS);
pros::ADIDigitalIn limit_switch('e');

double speed = 0;
double target_angle = 0;
double current_angle = 0;

void move(double speed) {
    motor.move(speed);
    turret::speed = speed;
}

const double LIMIT = 5.79;
const double RANGE = 138.35;

// create function to return motor position
double get_position() {
    return motor.get_position();
}

double get_angle() {
    return (motor.get_position() / (2 * LIMIT / RANGE));
}

void move_angle(double angle, double velocity) {
    double target_position = angle * (2 * LIMIT / RANGE);
    motor.move_absolute(target_position, velocity);
}

void home() {
    motor.move(80);

    while(!limit_switch.get_value()) {
        pros::delay(20);
    }

    // Stop the motor so it doesn't break the ring gear
    motor.move(0);
    motor.move_relative(-6.0, 400);

    pros::delay(1000);
    motor.move(0);

    motor.tare_position();
    pros::delay(100);
}


} // turret

namespace disklift {
    pros::Motor lift_motor(21, pros::E_MOTOR_GEARSET_36, true, pros::E_MOTOR_ENCODER_DEGREES);
    double lift_pos[] = {-15, 55, 68, 78}; //(DEPRECATED) -JBH 2/1/23
    int deltaDown = 2;
    int newPos = 0;
    int i = 0; // (DEPRECATED) -JBH 2/1/23
    double liftDownPos = 3;
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
        if(!(lift_motor.get_position() > liftDownPos)){
            // Acceptable tolerance, avoid burnout
            lift_motor.move_voltage(0);
        } else{
            lift_motor.move_absolute(liftDownPos,100);
        }
    }
    void home() {
        lift_motor.move(-80);
        int timestamp = pros::millis();
        while (lift_motor.get_current_draw() < 1000 && pros::millis() - timestamp < 2000) {
            pros::delay(10);
        }
        lift_motor.tare_position();
    }
}

namespace flywheel {
// flywheel tuning
int threshold = 14;
double kV = 60;
double kP = 0.5;
double kI = 0.000;
double kD = 0.1;

sylib::SpeedControllerInfo motor_speed_controller (
    [](double rpm){return kV;}, // kV function - 120
    kP, // kP - 1
    kI, // kI
    kD, // kD - 0.5
    0, // kH
    true, // anti-windup enabled
    3, // anti-windup range
    false, // p controller bounds threshold enabled
    3, // p controller bounds cutoff enabled - 5
    kP/4, // kP2 for when over threshold - 0.25
    threshold // range to target to apply max voltage - 10
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
    return speed - average < 1;
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
//deflector__________________________________________________________
namespace deflector {
int smart_port = 8;
char adi_port = 'c';
ADIDigitalOut deflector_piston({{smart_port,adi_port}});
bool state = true;
void toggle(){
    state = !state;
    deflector_piston.set_value(state);
}
}

//endgame__________________________________________________________
namespace endgame {
int smart_port = 8;
char adi_port = 'b';
ADIDigitalOut endgame_piston({{smart_port,adi_port}});
bool state = false;
void launch(){
    state = !state;
    endgame_piston.set_value(state);
}
}
