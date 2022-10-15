#include "main.h"
#include "pros/misc.h"
#include <cstdio>
// #include "api.h"
// #include "okapi/api.hpp"
#define DIGITAL_SENSOR_PORT 'A'

/////
// For instalattion, upgrading, documentations and tutorials, check out website!
// https://ez-robotics.github.io/EZ-Template/
/////

// Chassis constructor
Drive chassis(
    // Left Chassis Ports (negative port will reverse it!)
    //   the first port is the sensored port (when trackers are not used!)
    {11, -13}

    // Right Chassis Ports (negative port will reverse it!)
    //   the first port is the sensored port (when trackers are not used!)
    ,
    {-17, 14}

    // IMU Port
    ,
    20

    // Wheel Diameter (Remember, 4" wheels are actually 4.125!)
    //    (or tracking wheel diameter)
    ,
    3.25

    // Cartridge RPM
    //   (or tick per rotation if using tracking wheels)
    ,
    600

    // External Gear Ratio (MUST BE DECIMAL)
    //    (or gear ratio of tracking wheel)
    // eg. if your drive is 84:36 where the 36t is powered, your RATIO would
    // be 2.333. eg. if your drive is 36:60 where the 60t is powered, your RATIO
    // would be 0.6.
    ,
    0.6

    // Uncomment if using tracking wheels
    /*
    // Left Tracking Wheel Ports (negative port will reverse it!)
    // ,{1, 2} // 3 wire encoder
    // ,8 // Rotation sensor

    // Right Tracking Wheel Ports (negative port will reverse it!)
    // ,{-3, -4} // 3 wire encoder
    // ,-9 // Rotation sensor
    */

    // Uncomment if tracking wheels are plugged into a 3 wire expander
    // 3 Wire Port Expander Smart Port
    // ,1
);

// const int Cata_MOTOR_PORT = 1;
// pros::Motor cata(Cata_MOTOR_PORT);

// ControllerButton btnUp(ControllerDigital::R1);
// ControllerButton btnDown(ControllerDigital::R2);
pros::Motor cata(5);
pros::Motor intake(20);
// pros::Motor &intake_l = chassis.left_motors[1];
// pros::Motor &intake_r = chassis.right_motors[1];
// pros::ADIDigitalOut pto_intake_piston('A');
// bool pto_intake_enabled = false;

// void pto_intake(bool toggle) {
//   pto_intake_enabled = toggle;
//   chassis.pto_toggle({intake_l, intake_r}, toggle);
//   pto_intake_piston.set_value(toggle);
//   if (toggle) {
//     intake_l.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
//     intake_r.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
//   }
// }

// void set_intake(int input) {
//   if (!pto_intake_enabled)
//     return;
//   intake_l = input;
//   intake_r = input;
// }

// int button_lock = 0;
// void intake_control() {
//   if (master.get_digital(DIGITAL_DOWN) && button_lock == 0) {
//     pto_intake(!pto_intake_enabled);
//     button_lock = 1;
//   } else if (!master.get_digital(DIGITAL_DOWN)) {
//     button_lock = 0;
//   }

//   if (master.get_digital(DIGITAL_R1))
//     set_intake(127);
//   else if (master.get_digital(DIGITAL_R2))
//     set_intake(-127);
//   else
//     set_intake(0);
// }

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  // Print our branding over your terminal :D
  ez::print_ez_template();

  pros::delay(
      500); // Stop the user from doing anything while legacy ports configure.

  // Configure your chassis controls
  chassis.toggle_modify_curve_with_controller(
      true); // Enables modifying the controller curve with buttons on the
             // joysticks
  chassis.set_active_brake(0); // Sets the active brake kP. We recommend 0.1.
  chassis.set_curve_default(
      0, 0); // Defaults for curve. If using tank, only the first parameter is
             // used. (Comment this line out if you have an SD card!)
  default_constants(); // Set the drive to your own constants from autons.cpp!
  exit_condition_defaults(); // Set the exit conditions to your own constants
                             // from autons.cpp!

  // These are already defaulted to these buttons, but you can change the
  // left/right curve buttons here! chassis.set_left_curve_buttons
  // (pros::E_CONTROLLER_DIGITAL_LEFT, pros::E_CONTROLLER_DIGITAL_RIGHT); // If
  // using tank, only the left side is used.
  // chassis.set_right_curve_buttons(pros::E_CONTROLLER_DIGITAL_Y,
  // pros::E_CONTROLLER_DIGITAL_A);

  // Autonomous Selector using LLEMU
  ez::as::auton_selector.add_autons({
      Auton("Example Drive\n\nDrive forward and come back.", drive_example),
      Auton("Example Turn\n\nTurn 3 times.", turn_example),
      Auton("Drive and Turn\n\nDrive forward, turn, come back. ",
            drive_and_turn),
      Auton("Drive and Turn\n\nSlow down during drive.",
            wait_until_change_speed),
      Auton("Swing Example\n\nSwing, drive, swing.", swing_example),
      Auton("Combine all 3 movements", combining_movements),
      Auton("Interference\n\nAfter driving forward, robot performs differently "
            "if interfered or not.",
            interfered_example),
  });

  // Initialize chassis and auton selector
  chassis.initialize();
  ez::as::initialize();
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
  // . . .
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
  // . . .
}

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
void autonomous() {
  chassis.reset_pid_targets();               // Resets PID targets to 0
  chassis.reset_gyro();                      // Reset gyro position to 0
  chassis.reset_drive_sensor();              // Reset drive sensors to 0
  chassis.set_drive_brake(MOTOR_BRAKE_HOLD); // Set motors to hold.  This helps
                                             // autonomous consistency.

  // ez::as::auton_selector.call_selected_auton(); // Calls selected auton from
  // autonomous selector. set_tank(127, 127); pros::delay(1000); set_tank(0, 0);

  chassis.set_tank(127, 127);
  pros::delay(1000); // Wait 1 second
  chassis.set_tank(0, 0);

  // chassis.set_drive_pid(24, 100, true);
  // chassis.wait_drive();

  // chassis.set_drive_pid(-12, 100);
  // chassis.wait_drive();

  // chassis.set_drive_pid(-12, 100);
  // chassis.wait_drive();
}

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
  // This is preference to what you like to drive on.
  chassis.set_drive_brake(MOTOR_BRAKE_COAST);

  bool intakeOn = false;
  bool rollerOn = false;
  bool shooting = false;

  int right_y = master.get_analog(ANALOG_RIGHT_Y);

  while (true) {

    // chassis.tank(); // Tank control
    // chassis.arcade_standard(ez::SPLIT); // Standard split arcade

    // int l_stick =
    // chassis.left_curve_function(master.get_analog(ANALOG_LEFT_Y)); int
    // r_stick =
    //     chassis.left_curve_function(master.get_analog(ANALOG_RIGHT_Y));

    // chassis.set_tank(l_stick, r_stick);

    chassis.arcade_standard(ez::SINGLE);
    chassis.toggle_modify_curve_with_controller(true);

    if (master.get_digital(DIGITAL_L1)) {
      chassis.set_tank(0.1 * master.get_analog(ANALOG_LEFT_X),
                       0.1 * master.get_analog(ANALOG_LEFT_Y));
      // 1/3 speed
      // chassis.joy_thresh_opcontrol(
      //     (int)(0.2 * master.get_analog(ANALOG_LEFT_Y)),
      //     (int)(0.2 * master.get_analog(ANALOG_RIGHT_Y)));

      // chassis.modify_curve_with_controller();
    }
    // chassis.arcade_standard(ez::SINGLE); // Standard single arcade
    // chassis.arcade_flipped(ez::SPLIT); // Flipped split arcade
    // chassis.arcade_flipped(ez::SINGLE); // Flipped single arcade

    // . . .
    // Put more user control code here!
    // . . .

    // printf("Right Velocity: %i \n", chassis.right_velocity());

    if (master.get_digital(DIGITAL_R2)) {
      // cata.move_voltage(-3000);
      cata.move_relative(-1080 * 8.33, 100);
      // cata.move_relative(-680 * 8.33, 100);
      // cata.move_absolute(0, 100);
    }

    // move to 0
    if (master.get_digital(DIGITAL_B)) {
      cata.move_absolute(0, 100);
    }

    // set cata intake point
    if (master.get_digital(DIGITAL_Y)) {
      cata.set_zero_position(cata.get_position());
    }

    // move cata up/down override
    int new_right_y = master.get_analog(ANALOG_RIGHT_Y);
    if (new_right_y != right_y) {
      right_y = new_right_y;
      cata.move_voltage(right_y * 30);
    }

    // if (master.get_digital(DIGITAL_R2)) {
    //   // cata.move_relative(800, 100);
    //   cata.move_velocity(100);
    //   // cata.move_absolute(360, 100);
    // }

    if (master.get_digital(DIGITAL_R1)) {
      // intake
      intakeOn = !intakeOn;
      rollerOn = false;
      intake.move_velocity(100);
      // pros::delay(10);
    } else if (master.get_digital(DIGITAL_A)) {
      // outtake/roller
      intakeOn = false;
      rollerOn = !rollerOn;
      intake.move_velocity(-100);
      // pros::delay(10);
    } else {
      intake.move_velocity(0);
    }

    // if (intakeOn) {
    //   intake.move_velocity(100);
    // } else if (rollerOn) {
    //   intake.move_velocity(-100);
    // } else {
    //   intake.move_velocity(0);
    // }

    // pneumatic release
    if (master.get_digital(DIGITAL_DOWN)) {
      pros::ADIDigitalOut piston(DIGITAL_SENSOR_PORT);

      piston.set_value(true);
      pros::delay(1000);
      piston.set_value(false);
    }

    // cata
    // if (master.get_digital(DIGITAL_R2)) {
    //   cata.move_velocity(600);
    // } else {
    //   cata.move_velocity(0);
    // }

    // intake_control();

    pros::delay(10);
    // pros::delay(ez::util::DELAY_TIME); // This is used for timer
    // calculations! Keep this ez::util::DELAY_TIME
  }
}
