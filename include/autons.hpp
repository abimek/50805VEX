#pragma once

#include "EZ-Template/drive/drive.hpp"

extern Drive chassis;
extern pros::Motor intake;
extern pros::ADIDigitalOut intakelev;
extern pros::Controller master;
extern pros::ADIDigitalOut wing1;
extern pros::ADIDigitalOut wing2;

void auton1();

void auton_far_elim();
void auton_far_elim_2();
void auton_far_wp();
void auton_close_wp();

void drive_example();
void turn_example();
void drive_and_turn();
void wait_until_change_speed();
void swing_example();
void combining_movements();
void interfered_example();

void default_constants();
void one_mogo_constants();
void two_mogo_constants();
void exit_condition_defaults();
void modified_exit_condition();