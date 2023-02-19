#include "main.h"
#include "math.h"
#include "pros/adi.hpp"
#include "pros/misc.hpp"
#include "pros/motors.h"
#include "subsystems.h"
#include <cmath>

// intake -------------------------------------------------------------------------
namespace intake {

//LOCAL DEFS:
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

//LOCAL DEFS:
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

void set_brake_mode(pros::motor_brake_mode_e mode) {
    motor.set_brake_mode(mode);
}

} // roller

namespace disklift {
    
    //LOCAL DEFS:
    pros::Motor lift_motor(21, pros::E_MOTOR_GEARSET_36, true, pros::E_MOTOR_ENCODER_DEGREES);
    bool lifted = false; //if true, keep true until a disc is fired
    bool reachedSpeed = false;
    int targState = 0; // 0 = down, 1 = up, 2 = hold
    double liftDownPos = 7;

    void discLiftUp(){
        lift_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        //Prevents lifted from changing back to false momentariily, once it's set it stays until 
        //lifted AND flyweel detects a shot
        if(!lifted){
            lifted = lift_motor.get_actual_velocity() < 2 && lift_motor.get_position() > 12;
        } else if (lift_motor.get_actual_velocity() > 2){
            lifted = false;
        }
        if(!reachedSpeed){
            reachedSpeed = flywheel::at_speed();
        } else if (!flywheel::at_speed() && lifted){
            //Status changed + DL was up; shot detected
            lifted = false;
            reachedSpeed = false;
        }
        if(lifted){
            //DISC LIFT ALL THE WAY UP FOR CURRENT NUM OF DISCS
            lift_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
            lift_motor.brake();
        } else if(lift_motor.get_position() < 89){
            lift_motor.move_voltage(12000);
        } else{
            lift_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
            lift_motor.brake();
        }
    }
    
    void calculatePos(){
        // newPos = lift_motor.get_position();
    }

    void discLiftHold(){
        lift_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        if(lift_motor.get_position() < 89){
            lift_motor.move_voltage(6000);
            // lift_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
            // lift_motor.brake();
        } else{
            lift_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
            lift_motor.brake();
        }
    }

    void discLiftDown(){
        lifted = false;
        reachedSpeed = false;
        lift_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        if(lift_motor.get_position() < liftDownPos + 2){
            // Acceptable tolerance, avoid burnout
            lift_motor.move_voltage(0);
        } else{
            lift_motor.move_absolute(liftDownPos,100);
        }
    }

    void home() {
        lift_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
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
double speed = 0;



void move(double speed) {
    flywheel::speed = speed;
}

#define SMA_LEN 5

static double sma_data[SMA_LEN];
static int count=0;
static double average;

bool at_speed() {
    return std::abs(speed - average) / speed < 0.03;
}

void wait_until_fired() {
    while (speed - average < 20) {
        printf("wait_until_fired\n");
        pros::delay(10);
    }
}

void wait_until_at_speed() {
    while (!at_speed()) {
        printf("wait_until_at_speed\n");
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
        printf("%.2f,%.2f,%.2f\n",average, speed, left_flywheel.get_applied_voltage()/80.0);
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
int smart_port = 20;
char adi_port = 'f';
ADIDigitalOut endgame_piston('f');
bool state = false;
void launch(){
    state = !state;
    endgame_piston.set_value(state);
    std::cout << "Endgame launched" << std::endl;
}
} // namespace endgame

//misc__________________________________________________________
// Global shit like isGoldy
bool isSilva(){
FILE* usd_file_read = fopen("/usd/TURRET_ID.txt", "r");
char buf[50]; // This just needs to be larger than the contents of the file
fread(buf, 1, 50, usd_file_read); // passing 1 because a `char` is 1 byte, and 50 b/c it's the length of buf
int isGoldy = buf[0] == '1'; // buf[0] is the first character in the file, if it's a 1, the turret is goldy. If not, it's silva.
fclose(usd_file_read); 
return !isGoldy;
}
