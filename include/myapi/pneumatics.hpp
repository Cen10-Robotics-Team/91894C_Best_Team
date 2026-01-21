#pragma once

#include "api.h"

inline pros::adi::Pneumatics scoring_piston('a', false);
inline pros::adi::Pneumatics left_descore_piston('b', false);
inline pros::adi::Pneumatics right_descore_piston('c', false);
inline pros::adi::Pneumatics wall_load_piston('d', false);

void activate_upper_scoring();
void activate_mid_scoring();
void activate_wall_loading();
void deactivate_wall_loading();
void activate_left_descore();
void activate_right_descore();
void deactivate_left_descore();
void deactivate_right_descore();