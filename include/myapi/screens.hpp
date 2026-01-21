#pragma once
#include "api.h"
#include "robodash/api.h"
#include "myapi/auton.hpp"

LV_IMG_DECLARE(Huskytech);
LV_IMG_DECLARE(SecretPhoto);

inline rd::Image team_image(&Huskytech, "Team Image");
inline rd::Image secret_image(&SecretPhoto, "Secret Image");
inline rd::Console console;

inline rd::Selector selector({
    {"Right", blue_right},
    {"Left", blue_left},
    {"Right Long", blue_right_long},
    {"Left Long", blue_left_long},
    {"Auton Skills", auton_skills},
    {"Do Nothing", do_nothing_auton}
});

void coordinate_task(void* param);