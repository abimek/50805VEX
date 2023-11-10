#include "main.h"


/////
// For instalattion, upgrading, documentations and tutorials, check out website!
// https://ez-robotics.github.io/EZ-Template/
/////


// Chassis constructor
Drive chassis (
  // Left Chassis Ports (negative port will reverse it!)
  //   the first port is the sensored port (when trackers are not used!)
 // {12, 13, 14}
   {-12, -13, -14}


  // Right Chassis Ports (negative port will reverse it!)
  //   the first port is the sensored port (when trackers are not used!)
  //,{20, 18, 17}
  ,{16, 18, 17}

  // IMU Port
  // TODO: Mount IMU
  ,21

  // Wheel Diameter (Remember, 4" wheels are actually 4.125!)
  //    (or tracking wheel diameter)
  ,2.75

  // Cartridge RPM
  //   (or tick per rotation if using tracking wheels)
  ,600

  // External Gear Ratio (MUST BE DECIMAL)
  //    (or gear ratio of tracking wheel)
  // eg. if your drive is 84:36 where the 36t is powered, your RATIO would be 2.333.
  // eg. if your drive is 36:60 where the 60t is powered, your RATIO would be 0.6.
  ,1.333

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

//  PROS materials used for the robots movement on the map. 
pros::Motor fly_wheel(11);
pros::Motor blocker(15);
pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::ADIDigitalOut wings('A', false);
pros::ADIDigitalOut intake('B', false);
pros::ADIDigitalOut ratchet('C', false);

bool flywheel_on = false;
bool wings_on = false;
bool intake_on = false;
bool ratchet_on = false;

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  // Print our branding over your terminal :D

  ez::print_ez_template();
  
  pros::delay(500); // Stop the user from doing anything while legacy ports configure.

  // Configure your chassis controls
  chassis.toggle_modify_curve_with_controller(true); // Enables modifying the controller curve with buttons on the joysticks
  chassis.set_active_brake(0.1); // Sets the active brake kP. We recommend 0.1.
  chassis.set_curve_default(0, 0); // Defaults for curve. If using tank, only the first parameter is used. (Comment this line out if you have an SD card!)  
  default_constants(); // Set the drive to your own constants from autons.cpp!
  exit_condition_defaults(); // Set the exit conditions to your own constants from autons.cpp!

  // These are already defaulted to these buttons, but you can change the left/right curve buttons here!
  // chassis.set_left_curve_buttons(pros::E_CONTROLLER_DIGITAL_LEFT, pros::E_CONTROLLER_DIGITAL_RIGHT); // If using tank, only the left side is used. 
  // chassis.set_right_curve_buttons(pros::E_CONTROLLER_DIGITAL_Y,    pros::E_CONTROLLER_DIGITAL_A);

  // Autonomous Selector using LLEMU
  ez::as::auton_selector.add_autons({
    Auton("Far Elim", auton_far_elim),
    Auton("Far WP", auton_far_wp),
    Auton("Close WP", auton_close_wp),
    Auton("Example Drive\n\nDrive forward and come back.", drive_example),
   /* Auton("Example Turn\n\nTurn 3 times.", turn_example),
    Auton("Offense Auton\n\nTurn 3 times.", auton_offense),
    Auton("Drive and Turn\n\nDrive forward, turn, come back. ", drive_and_turn),
    Auton("Drive and Turn\n\nSlow down during drive.", wait_until_change_speed),
    Auton("Swing Example\n\nSwing, drive, swing.", swing_example),
    Auton("Combine all 3 movements", combining_movements),
    Auton("Interference\n\nAfter driving forward, robot performs differently if interfered or not.", interfered_example),*/
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
  chassis.reset_pid_targets(); // Resets PID targets to 0
  chassis.reset_gyro(); // Reset gyro position to 0
  chassis.reset_drive_sensor(); // Reset drive sensors to 0
  chassis.set_drive_brake(MOTOR_BRAKE_HOLD); // Set motors to hold.  This helps autonomous consistency.

  ez::as::auton_selector.call_selected_auton(); // Calls selected auton from autonomous selector.
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
 * task, not resume it from where it left off
 */
void opcontrol() {
  // This is preference to what you like to drive on.
  flywheel_on = false;
  wings_on = false;
  intake_on = false;
  ratchet_on = false;
  wings.set_value(wings_on);
  ratchet.set_value(ratchet_on);
  intake.set_value(intake_on);
  fly_wheel.move(0);
  blocker.set_brake_mode(MOTOR_BRAKE_HOLD);
  chassis.set_drive_brake(MOTOR_BRAKE_COAST);
  while (true) {

    bool flywheel_toggle = master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A) == 1;
    if(flywheel_toggle){
      flywheel_on = !flywheel_on;
    }
    chassis.tank(); // Tank control
    if (flywheel_on) {
      fly_wheel.move(-127);
    }else{
      fly_wheel.move(0);
    }

    bool wings_toggle = master.get_digital_new_press(DIGITAL_L2) == 1;
    if(wings_toggle){
      wings_on = !wings_on;
      wings.set_value(wings_on);
    }

    bool ratchet_toggle = master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y) == 1;
    if(ratchet_toggle){
      ratchet_on = !ratchet_on;
      ratchet.set_value(ratchet_on);
    }


    bool intake_toggle = master.get_digital_new_press(DIGITAL_L1) == 1;
    if(intake_toggle){
      intake_on = !intake_on;
      intake.set_value(intake_on);
    }
    

    if(master.get_digital(DIGITAL_R1)){
      blocker.move(-127);
    }else if (master.get_digital(DIGITAL_R2)){
      blocker.move(127);
    }else{
      blocker.move(0);
    }


    // chassis.arcade_standard(ez::SPLIT); // Standard split arcade
    // chassis.arcade_standard(ez::SINGLE); // Standard single arcade
    // chassis.arcade_flipped(ez::SPLIT); // Flipped split arcade
    // chassis.arcade_flipped(ez::SINGLE); // Flipped single arcade

    // . . .
    // Put more user control code here!
    // . . .

    pros::delay(ez::util::DELAY_TIME); // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}
