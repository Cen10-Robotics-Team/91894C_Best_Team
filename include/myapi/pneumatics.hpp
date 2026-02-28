#pragma once

#include "api.h"

inline pros::adi::Pneumatics scoring_piston('a', false);
inline pros::adi::Pneumatics mid_descore_piston('b', true);
inline pros::adi::Pneumatics right_descore_piston('c', false);
inline pros::adi::Pneumatics wall_load_piston('d', false);

void activate_upper_scoring();
void activate_mid_scoring();
void activate_wall_loading();
void deactivate_wall_loading();
void activate_mid_descore();
void activate_right_descore();
void deactivate_mid_descore();
void deactivate_right_descore();