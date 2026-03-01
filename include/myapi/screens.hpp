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
    {"Move Forward", do_nothing_auton},
    {"Blue Right", blue_right},
    {"Blue Right Long", blue_right_long},
    {"Blue Right AWP", blue_right_awp},
    {"Blue Left", blue_left},
    {"Blue Left Long", blue_left_long},
    {"Blue Left AWP", blue_left_awp},
    {"Red Right", red_right},
    {"Red Right Long", red_right_long},
    {"Red Right AWP", red_right_awp},
    {"Red Left", red_left},
    {"Red Left Long", red_left_long},
    {"Red Left AWP", red_left_awp},
    {"Auton Skills", auton_skills},
    
});

void coordinate_task(void* param);