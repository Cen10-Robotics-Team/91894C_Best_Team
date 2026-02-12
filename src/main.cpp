#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep

pros::Controller controller(pros::E_CONTROLLER_MASTER);
float errorMoves[10] = {5, 10, 15, 20, 25, 30, 35, 40, 45, 48};
int i = 0;


/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */


void initialize() {
    chassis.calibrate();
    selector.focus();
    //chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    //pros::Task run_auto_rejector(auto_reject);
    //pros::Task run_stop_intake_stalling(stop_intake_stalling);
    pros::Task run_coordinate_task(coordinate_task);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
    team_image.focus();
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
    selector.focus();
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */



void autonomous() {
    //selector.run_auton();
    blue_right_awp();
    //activate_intake(false);
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */





void opcontrol() {
    //hello test

    // loop forever
    
    while (true) {
        

        // get left y and right x positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // move the robot
        chassis.arcade(leftY, rightX);

        if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            stop_scoring();
            activate_intake(true);
        }

        if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
            stop_intake();
        }

        if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
            stop_all_intake_motors();
            scoring_piston.retract();
            
        }

        if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
            score_intake("high");
        }

        if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
            score_intake("mid");
        }

        if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            score_intake("low");
        }

        /*if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
            //toggle_auto_reject();
            chassis.moveToPoint(chassis.getPose().x, errorMoves[i], 2000);
        }*/

        if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
            left_descore_piston.toggle();
        }

        if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
            scoring_piston.toggle();
        }

        if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) {
            right_descore_piston.toggle();
        }

        if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) {
            wall_load_piston.toggle();
        }

        // delay to save resources
        pros::delay(20); 
    }
}