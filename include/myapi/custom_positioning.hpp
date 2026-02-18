#pragma once
#include "api.h"

inline pros::Distance left(7);
inline pros::Distance right(8);

inline double sensors_gap = 20;

double get_theta_to_wall();

void printTheta(void* param);