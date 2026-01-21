#include "main.h"

void activate_upper_scoring() {
    scoring_piston.extend();
}

void activate_mid_scoring() {
    scoring_piston.retract();
}

void activate_wall_loading() {
    wall_load_piston.extend();
}

void deactivate_wall_loading() {
    wall_load_piston.retract();
}

void activate_left_descore() {
    left_descore_piston.extend();
}

void activate_right_descore() {
    right_descore_piston.extend();
}

void deactivate_left_descore() {
    left_descore_piston.retract();
}

void deactivate_right_descore() {
    right_descore_piston.retract();
}