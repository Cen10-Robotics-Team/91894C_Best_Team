#pragma once
#include "api.h"

inline pros::Motor intake_motor_1(20, pros::MotorGearset::green);
inline pros::Motor intake_motor_2(19, pros::MotorGearset::green);
inline pros::Motor scoring_motor(17, pros::MotorGearset::green);
inline pros::Optical color_sensor(10);

inline bool enable_auto_reject;
inline bool scoring;
inline bool intaking;


void activate_intake(bool direction);
void intake_balls();
void stop_intake();
void stop_scoring();
void stop_all_intake_motors();
void score_intake(std::string goal);
void reject_intake();
std::string detect_color();
void auto_reject();
void stop_intake_stalling();
void toggle_auto_reject();