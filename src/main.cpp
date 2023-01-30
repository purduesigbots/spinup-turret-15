#include "main.h"
#include "ARMS/config.h"
#include "subsystems.h"

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
  static bool pressed = false;
  pressed = !pressed;
  if (pressed) {
    pros::lcd::set_text(2, "I was pressed!");
  } else {
    pros::lcd::clear_line(2);
  }
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  arms::init();
  arms::odom::reset({0, 0}, 0.0); // start position
  pros::delay(2000);

  pros::lcd::initialize();
  pros::lcd::set_text(1, "Hello PROS User!");
  pros::lcd::set_background_color(LV_COLOR_BLACK);
  pros::lcd::set_text_color(LV_COLOR_WHITE);

  pros::lcd::register_btn1_cb(on_center_button);

  roller::init();
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

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
void autonomous() {}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
  pros::Controller master(pros::E_CONTROLLER_MASTER);
  using namespace arms::chassis;

  // move(30, 70);
  // move(-4, 50, arms::REVERSE);
  // turn(89);
  // move(-6, 50, arms::REVERSE);
  // move(5, 50, arms::REVERSE);
  // turn(51, arms::RELATIVE);
  // move(36, 70);
  // turn(-90, arms::RELATIVE);
  // move(5, 50);

  while (true) {

    int left = master.get_analog(ANALOG_LEFT_Y);
    int right = master.get_analog(ANALOG_RIGHT_X);
    arms::chassis::arcade(left, right);
    
    if (master.get_digital(DIGITAL_R1)) { // intake
      intake::move(100);
    } else if (master.get_digital(DIGITAL_R2)) { // outake
      intake::move(-100);
    } else { // idle
      intake::move(0);
    }

    if (master.get_digital(DIGITAL_L1)) { // Turret Left
      turret::move(100);
    } else if (master.get_digital(DIGITAL_L2)) { // Turret Right
      turret::move(-100);
    } else { // idle
      turret::move(0);
    }

    if (master.get_digital(DIGITAL_X)) { // intake
      roller::move(100);
    } else if (master.get_digital(DIGITAL_B)) { // outake
      roller::move(-100);
    } else { // idle
      roller::move(0);
    }

    if (master.get_digital_new_press(DIGITAL_A)) {
      roller::toggle_turn_roller();
    }

    // printf("left encoder %f\n", arms::odom::getLeftEncoder());
    // printf("right encoder %f\n", arms::odom::getRightEncoder());
    // printf("middle encoder %f\n", arms::odom::getMiddleEncoder());

    pros::delay(20);
  }
}
