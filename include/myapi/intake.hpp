#pragma once
#include "api.h"

inline pros::Motor intake_motor_1(20, pros::MotorGearset::green);
inline pros::Motor intake_motor_2(19, pros::MotorGearset::green);
inline pros::Motor scoring_motor(17, pros::MotorGearset::green);
inline pros::Optical midgoal_color_sensor(11);
inline pros::Optical highgoal_color_sensor(10);

inline bool enable_auto_reject;
inline bool high_scoring;
inline bool mid_scoring;
inline bool low_scoring;
inline bool intaking;

void activate_intake(bool direction);
void stop_intake();
void stop_scoring();
void stop_all_intake_motors();
void score_intake(std::string goal);
void slow_mid_score();
void high_reject_intake();
void mid_reject_intake();
std::string detect_color_for_mid();
std::string detect_color_for_high();
void high_auto_reject(void *);
void mid_auto_reject(void *);
void stop_intake_stalling();
void toggle_auto_reject();