#include "main.h"

void activate_upper_scoring() { scoring_piston.retract(); }

void activate_mid_scoring() { scoring_piston.extend(); }

void activate_wall_loading() { wall_load_piston.extend(); }

void deactivate_wall_loading() { wall_load_piston.retract(); }

void activate_mid_descore() { mid_descore_piston.extend(); }

void activate_right_descore() { right_descore_piston.retract(); }

void deactivate_mid_descore() { mid_descore_piston.extend(); }

void deactivate_right_descore() { right_descore_piston.retract(); }