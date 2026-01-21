#include "main.h"

void coordinate_task(void* param) {
    while (true) {
        console.println("x: " + std::to_string(chassis.getPose().x));
        console.println("y: " + std::to_string(chassis.getPose().y));
        console.println("theta: " + std::to_string(chassis.getPose().theta));

        pros::delay(50);
        console.clear();
    }
}