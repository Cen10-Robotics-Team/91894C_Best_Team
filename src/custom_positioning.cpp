#include "main.h"
#include <cmath>
#include <numbers>
#include <cstdlib>

double get_theta_to_wall() {
    int left_distance = left.get();
    int right_distance = right.get();

    double diff_over_distance = (left_distance - right_distance) / sensors_gap;

    double theta_to_wall_rad = std::atan(diff_over_distance);

    double theta_to_wall_deg = theta_to_wall_rad * (180.0 / std::numbers::pi);

    return theta_to_wall_deg;
}