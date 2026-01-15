#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "liblvgl/widgets/lv_img.h"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/motors.hpp"
#include <string>
// Changed the left motors to be all the same direction and right motors in the same direction as well. Be happy. -normalperson543
pros::MotorGroup left_motors({-1, -2, -3}, pros::MotorGearset::blue); // left motors on ports 1, 2, 3 
pros::MotorGroup right_motors({4, 5, 6}, pros::MotorGearset::blue); // right motors on ports 4, 5, 6
// drivetrain settings
lemlib::Drivetrain drivetrain(&left_motors, // left motor group
                              &right_motors, // right motor group
                              12.8011811, // 12.8 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
                              360, // drivetrain rpm is 360
                              2 // horizontal drift is 2 (for now)
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttle_curve(15, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.003 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steer_curve(15, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.003 // expo curve gain
);

//inertial sensor on port 11
pros::Imu imu(11);

// create v5 rotation sensors on ports 12 & 13
pros::Rotation rotation_sensor_horizontal(-13);
pros::Rotation rotation_sensor_vertical(-12);

// horizontal tracking wheel
lemlib::TrackingWheel horizontal_tracking_wheel(&rotation_sensor_horizontal, lemlib::Omniwheel::NEW_2, 1.25937008);

// vertical tracking wheel
lemlib::TrackingWheel vertical_tracking_wheel(&rotation_sensor_vertical, ::lemlib::Omniwheel::NEW_2, 1.2330709);

// Odom Sensors
lemlib::OdomSensors sensors(&vertical_tracking_wheel, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            &horizontal_tracking_wheel, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(14.1, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              38, // derivative gain (kD)
                                              3, // anti windup 3
                                              1, // small error range, in inches 1
                                              100, // small error range timeout, in milliseconds 100
                                              3, // large error range, in inches 3
                                              500, // large error range timeout, in milliseconds 500
                                              20 // maximum acceleration (slew) 20
);

// angular PID controller
lemlib::ControllerSettings angular_controller(2.5, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              15, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew) // maximum acceleration (slew)
);
//chassis definition
lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors,
                        &throttle_curve,
                        &steer_curve // odometry sensors
);

pros::Motor intake_motor_1(20, pros::MotorGearset::green);
pros::Motor intake_motor_2(19, pros::MotorGearset::green);
pros::Motor intake_motor_3(18, pros::MotorGearset::green);

pros::Motor scoring_motor(17, pros::MotorGearset::green);
pros::adi::Pneumatics scoring_piston('a', false);
pros::adi::Pneumatics left_descore_piston('b', false);
pros::adi::Pneumatics right_descore_piston('c', false);
pros::adi::Pneumatics wall_load_piston('d', false);

pros::Optical color_sensor(10);

pros::Controller controller(pros::E_CONTROLLER_MASTER);

LV_IMG_DECLARE(Huskytech);
LV_IMG_DECLARE(SecretPhoto);

rd::Image team_image(&Huskytech, "Team Image");
rd::Image secret_image(&SecretPhoto, "Secret Image");

rd::Console console;

bool enable_auto_reject = false;
std::string alliance_color = "";
bool scoring = false;
bool intaking = false;

void activate_upper_scoring() {
    scoring_piston.retract();
}

void activate_mid_scoring() {
    scoring_piston.extend();
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

void activate_intake(bool direction){
    if(direction) {
        intake_motor_1.move(127);
        intake_motor_2.move(127);
        intake_motor_3.move(127);

    } else {
        intake_motor_1.move(-127);  
        intake_motor_2.move(-127);
        intake_motor_3.move(-127);
        
    }
    intaking = true;
}

void intake_balls() {
    intake_motor_1.move(127);
    intake_motor_2.move(127);

    intaking = true;
}

void stop_intake() {
    intake_motor_1.brake();
    intake_motor_2.brake();
    intake_motor_3.brake();

    intaking = false;
}

void stop_scoring() {
    scoring_motor.brake();

    scoring = false;
}

void stop_all_intake_motors() {
    stop_intake();
    stop_scoring();
}

void score_intake(std::string goal) {
    if(goal == "low") {
        activate_upper_scoring();
        scoring_motor.move(127);
        activate_intake(false);
        scoring = true;
    } else if (goal == "mid") {
        activate_mid_scoring();
        scoring_piston.retract();
        scoring_motor.move(-127);
        pros::delay(200);
        intake_motor_3.move(127);
        pros::delay(200);
        intake_motor_2.move(127);
        pros::delay(200);
        intake_motor_1.move(127);
        scoring = true;
        intaking = true;
    } else if (goal == "high") {
        activate_upper_scoring();
        scoring_motor.move(-127);
        pros::delay(200);
        intake_motor_3.move(127);
        pros::delay(200);
        intake_motor_2.move(127);
        pros::delay(200);
        intake_motor_1.move(127);
        scoring = true;
        intaking = true;
    }
}

void reject_intake() {
    score_intake("mid");
    pros::delay(100);
    stop_scoring();
}

std::string detect_color() {
    int rgb_value = color_sensor.get_hue();
    if(rgb_value >= 180 && rgb_value <= 215) {
        return "blue";
    } else if (rgb_value <= 15) {
        return "red";
    } else {
        return "";
    }
}

void auto_reject() {
    while(true) {
        while (enable_auto_reject) {
            std::string block_color = detect_color();
            if(alliance_color == "red" && block_color == "blue" && color_sensor.get_proximity() > 200) {
                reject_intake();
            } else if (alliance_color == "blue" && block_color == "red" && color_sensor.get_proximity() > 200) {
                reject_intake();
            }
        }

        pros::delay(20);
    }
}

void stop_intake_stalling() {
    while (true) {
        if(intaking) {
            if(intake_motor_1.get_current_draw() > 2.0 && intake_motor_1.get_actual_velocity() < 10) {
                int target_velocity = intake_motor_1.get_target_velocity();
                intake_motor_1.move(-1 * target_velocity);
                pros::delay(50);
                intake_motor_1.move(target_velocity);
            }

            if(intake_motor_2.get_current_draw() > 2.0 && intake_motor_2.get_actual_velocity() < 10) {
                int target_velocity = intake_motor_2.get_target_velocity();
                intake_motor_2.move(-1 * target_velocity);
                pros::delay(50);
                intake_motor_2.move(target_velocity);
            }

            if(intake_motor_3.get_current_draw() > 2.0 && intake_motor_3.get_actual_velocity() < 10) {
                int target_velocity = intake_motor_3.get_target_velocity();
                intake_motor_3.move(-1 * target_velocity);
                pros::delay(50);
                intake_motor_3.move(target_velocity);
            }
        }

        if(scoring) {
            if(scoring_motor.get_current_draw() > 2.0 && scoring_motor.get_actual_velocity() < 5) {
                int target_velocity = scoring_motor.get_target_velocity();
                scoring_motor.move(-1 * target_velocity);
                pros::delay(50);
                scoring_motor.move(target_velocity);
            }
        }
    }
}

void toggle_auto_reject() {
    enable_auto_reject = !enable_auto_reject;
}

void red_right_awp() {
    //not tuned
    chassis.setPose(0, 0, 0);
    alliance_color = "red";
    console.println("This is red right awp");
    chassis.moveToPoint(-42, 26, 2000, {.maxSpeed = 96});
    chassis.turnToHeading(-135, 500, {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE});
    chassis.moveToPoint(-30, 37, 750, {.forwards = false, .maxSpeed = 96}, false);
    pros::delay(1000);
    chassis.moveToPoint(-32, 35, 500);
    chassis.moveToPoint(7, 28, 1500, {.maxSpeed = 112});
    chassis.turnToHeading(-45, 750, {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE});
    chassis.moveToPoint(-3, 36, 750, {}, false);
    pros::delay(1000);
    chassis.moveToPoint(28, 0, 1500, {.forwards = false, .maxSpeed = 96});
    chassis.turnToHeading(180, 750, {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE});
    chassis.moveToPoint(28, -10, 500, {}, false);
    pros::delay(1000);
    chassis.moveToPoint(31, 12, 750, {.forwards = false, .maxSpeed = 100}, false);
}

void red_right() {
    chassis.setPose(0, 0, 0);
    alliance_color = "red";
    console.println("This is red right ");
    activate_intake(true);
    chassis.moveToPoint(2.5, 27, 1000, {.minSpeed = 24, .earlyExitRange = 1});
    chassis.moveToPoint(6.5, 46.5, 3000, {.maxSpeed = 24});
    chassis.turnToPoint(-1.5, 52, 500, {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE});
    chassis.moveToPoint(-1.5, 52, 1000, {}, false);
    score_intake("low");
    pros::delay(2000);
    stop_all_intake_motors();
    chassis.moveToPoint(31, 15, 1750, {.forwards = false, .maxSpeed = 112}, false);
    activate_wall_loading();
    chassis.turnToHeading(180, 500, {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE});
    activate_intake(true);
    chassis.moveToPoint(31, 5, 1000, {.maxSpeed = 64}, false);
    pros::delay(500);
    chassis.moveToPoint(31, 37, 1500, {.forwards = false, .maxSpeed = 100}, false);
    deactivate_wall_loading();
    score_intake("high");
    pros::delay(2000);
    stop_all_intake_motors();
    chassis.moveToPoint(31, 27, 1000, {.maxSpeed = 112});
    
    chassis.turnToHeading(-90, 500);
    chassis.moveToPoint(20.5, chassis.getPose().y, 1000);
    chassis.turnToHeading(0, 500, {});
    activate_right_descore();
}

void red_left_awp() {
    //not tuned
    chassis.setPose(0, 0, 0);
    alliance_color = "red";
    console.println("This is red left awp");
    chassis.moveToPoint(42, 26, 2000, {.maxSpeed = 96});
    chassis.turnToHeading(-45, 500, {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE});
    chassis.moveToPoint(30, 37, 750, {.maxSpeed = 96}, false);
    pros::delay(1000);
    chassis.moveToPoint(45, 22, 500, {.forwards = false});
    chassis.moveToPoint(-10, 25, 1500, {.maxSpeed = 96});
    chassis.turnToHeading(-135, 750, {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE});
    chassis.moveToPoint(3, 36, 750, {.forwards = false}, false);
    pros::delay(1000);
    chassis.moveToPoint(-34, 0, 1500, {.maxSpeed = 96});
    chassis.turnToHeading(180, 750, {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE});
    chassis.moveToPoint(-34, -10, 500, {}, false);
    pros::delay(1000);
    chassis.moveToPoint(-33, 12, 750, {.forwards = false, .maxSpeed = 100}, false);
    pros::delay(1000);
}

void red_left() {
    chassis.setPose(0, 0, 0);
    alliance_color = "red";
    console.println("This is red left");
    activate_intake(true);
    chassis.moveToPoint(-3.5, 12, 750, {.minSpeed = 24, .earlyExitRange = 1});
    chassis.moveToPoint(-7, 30.25, 4000, {.maxSpeed = 24});
    chassis.turnToHeading(225, 500, {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE});
    chassis.moveToPoint(4, 37, 1000, {.forwards = false}, false);
    score_intake("mid");
    activate_wall_loading();
    pros::delay(2000);
    stop_all_intake_motors();
    chassis.moveToPoint(-32.5, 5, 1500, {.maxSpeed = 100});
    chassis.turnToHeading(180, 500, {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE});
    activate_intake(true);
    chassis.moveToPoint(-32.5, -10, 1000, {.maxSpeed = 64}, false);
    pros::delay(650);
    stop_intake();
    chassis.moveToPoint(-32.5, 21, 750, {.forwards = false, .maxSpeed = 112}, false);
    deactivate_wall_loading();
    score_intake("high");
    pros::delay(2000);
    stop_all_intake_motors();
    chassis.moveToPoint(-32.5, 11, 1000);
    
    chassis.turnToHeading(-90, 500);
    activate_right_descore();
    chassis.moveToPoint(-39, chassis.getPose().y, 1000);
    chassis.turnToHeading(0, 500, {}); 
}

void blue_right() {
    //right_long_plus_mid path
    chassis.setPose(-62.125, -16.935, 90);
    alliance_color = "blue";
    console.println("This is blue right long + mid ");
    activate_intake(true);
    chassis.moveToPoint(-34.377, -21.024, 750, {.minSpeed = 24, .earlyExitRange = 1});
    chassis.moveToPoint(-23.392, -22.729, 3000, {.maxSpeed = 24});
    chassis.turnToPoint(-12.785, -12.501, 500);
    chassis.moveToPoint(-12.785, -12.501, 750, {}, false);
    score_intake("low");
    pros::delay(1500);
    stop_all_intake_motors();
    chassis.moveToPoint(-47.125, -46.935, 1500, {.forwards = false, .maxSpeed = 112}, false);
    activate_wall_loading();
    chassis.turnToHeading(270, 500);
    activate_intake(true);
    chassis.moveToPoint(-60.125, -46.935, 1000, {.maxSpeed = 64}, false);
    pros::delay(500);
    chassis.moveToPoint(-30.125, -46.935, 1000, {.forwards = false, .maxSpeed = 100}, false);
    deactivate_wall_loading();
    score_intake("high");
    pros::delay(1500);
    stop_all_intake_motors();
    //chassis.moveToPoint(-35.125, -46.935, 500, {.maxSpeed = 112});
    
    //chassis.turnToHeading(0, 500);
    chassis.moveToPoint(-35.125, -35.935, 500);
    chassis.turnToHeading(90, 500, {});
    activate_right_descore();
    chassis.moveToPoint(-13.349, -35.935, 3000);
}

void blue_right_long_goal() {
    chassis.setPose(-62.125, -16.935, 90);
    alliance_color = "blue";
    console.println("This is blue right long");
    activate_intake(true);
    chassis.moveToPoint(-34.377, -21.024, 750, {.minSpeed = 24, .earlyExitRange = 1});
    chassis.moveToPoint(-23.392, -22.729, 3000, {.maxSpeed = 24});
    chassis.turnToPoint(-47.125, -46.935, 500);
    chassis.moveToPoint(-47.125, -46.935, 1500, {.maxSpeed = 112}, false);
    activate_wall_loading();
    chassis.turnToHeading(270, 500);
    activate_intake(true);
    chassis.moveToPoint(-60.125, -46.935, 1000, {.maxSpeed = 64}, false);
    pros::delay(500);
    chassis.moveToPoint(-30.125, -46.935, 1000, {.forwards = false, .maxSpeed = 100}, false);
    deactivate_wall_loading();
    score_intake("high");
    pros::delay(3000);
    stop_all_intake_motors();
    //chassis.moveToPoint(-35.125, -46.935, 500, {.maxSpeed = 112});
    
    //chassis.turnToHeading(0, 500);
    chassis.moveToPoint(-35.125, -35.935, 500);
    chassis.turnToHeading(90, 500, {});
    activate_right_descore();
    chassis.moveToPoint(-13.349, -35.935, 3000);
}
void blue_right_mid() {
    chassis.setPose(-62.125, -16.935, 90);
    alliance_color = "blue";
    console.println("This is blue right mid");
    activate_intake(true);
    chassis.moveToPoint(-34.377, -21.024, 750, {.minSpeed = 24, .earlyExitRange = 1});
    chassis.moveToPoint(-23.392, -22.729, 3000, {.maxSpeed = 24});
    chassis.turnToPoint(-47.125, -46.935, 500);
    chassis.moveToPoint(-47.125, -46.935, 1500, {.maxSpeed = 112}, false);
    activate_wall_loading();
    chassis.turnToHeading(270, 500);
    activate_intake(true);
    chassis.moveToPoint(-60.125, -46.935, 1000, {.maxSpeed = 64}, false);
    pros::delay(500);
    chassis.moveToPoint(-47.125, -46.935, 1500, {.forwards = false, .maxSpeed = 112});
    chassis.turnToPoint(-12.785, -12.501, 500);
    chassis.moveToPoint(-12.785, -12.501, 1500, {}, false);
    score_intake("low");
    pros::delay(3000);
    stop_all_intake_motors();
}

void blue_right_all_blocks() {
    chassis.setPose(-62.125, -16.935, 90);
    alliance_color = "blue";
    console.println("This is blue right all blocks");
    activate_intake(true);
    chassis.moveToPoint(-34.377, -21.024, 750, {.minSpeed = 24, .earlyExitRange = 1});
    chassis.moveToPoint(-23.392, -22.729, 2500, {.maxSpeed = 24});
    chassis.turnToPoint(-12.785, -12.501, 500);
    chassis.moveToPoint(-12.785, -12.501, 750, {}, false);
    score_intake("low");
    pros::delay(1500);
    stop_all_intake_motors();
    chassis.moveToPoint(-15.523, -15.16, 500, {.forwards = false});
    chassis.turnToPoint(-5.713, -41.182, 500);
    chassis.moveToPoint(-5.713, -41.182, 750);
    chassis.moveToPoint(-5.713, -43.182, 500);
    chassis.moveToPoint(-19.943, -35.953, 750, {.forwards = false});

    chassis.moveToPoint(-47.125, -46.935, 750, {.forwards = false, .maxSpeed = 112}, false);
    activate_wall_loading();
    chassis.turnToHeading(270, 500);
    activate_intake(true);
    chassis.moveToPoint(-60.125, -46.935, 750, {.maxSpeed = 64}, false);
    pros::delay(500);
    chassis.moveToPoint(-30.125, -46.935, 750, {.forwards = false, .maxSpeed = 100}, false);
    deactivate_wall_loading();
    score_intake("high");
    pros::delay(1500);
    stop_all_intake_motors();
    //chassis.moveToPoint(-35.125, -46.935, 500, {.maxSpeed = 112});
    
    //chassis.turnToHeading(0, 500);
    chassis.moveToPoint(-35.125, -35.935, 500);
    chassis.turnToHeading(90, 500, {});
    activate_right_descore();
    chassis.moveToPoint(-13.349, -35.935, 1000);
}

void blue_right_awp() {
    
    chassis.setPose(-62.125, -16.935, 90);
    alliance_color = "blue";
    console.println("This is blue right awp");
    activate_intake(true);
    chassis.moveToPoint(-47.125, -16.935, 500);
    chassis.turnToPoint(-28.256, 12.119, 500);
    chassis.moveToPoint(-28, 12.119, 750, {.maxSpeed = 96, .minSpeed = 32, .earlyExitRange = 1});
    chassis.moveToPoint(-22.103, 21.951, 2500, {.maxSpeed = 32});
    chassis.turnToHeading(-135, 500, {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE}, false);

    chassis.moveToPoint(-12.785, 12.501, 500, {.forwards = false, .maxSpeed = 96}, false);
    score_intake("mid");
    pros::delay(1000);
    stop_all_intake_motors();
    chassis.moveToPoint(-16.889, 16.468, 500);
    chassis.turnToPoint(-20.526, -11.399, 500);
    chassis.moveToPoint(-20.526, -11.399, 750, {.minSpeed = 32, .earlyExitRange = 1});
    chassis.moveToPoint(-21.953, -22.094, 2500, {.maxSpeed = 32});
    chassis.turnToPoint(-12.785, -12.501, 500);
    chassis.moveToPoint(-12.785, -12.501, 500, {}, false);
    score_intake("low");
    pros::delay(1000);
    stop_all_intake_motors();
    chassis.moveToPoint(-47.125, -46.841, 1000, {.forwards = false, .maxSpeed = 96});
    chassis.turnToHeading(270, 500, {}, false);
    activate_wall_loading();
    chassis.moveToPoint(-60.125, -46.935, 500, {});
    activate_intake(true);
    pros::delay(500);
    stop_intake();
    chassis.moveToPoint(-30.125, -46.935, 750, {.forwards = false, .maxSpeed = 100}, false);
    score_intake("high");

    chassis.moveToPoint(-35.125, -35.935, 500);
    chassis.turnToHeading(90, 500, {});
    activate_right_descore();
    chassis.moveToPoint(-13.349, -35.935, 1000);
}


void blue_left_awp() {
    //not tuned
    chassis.setPose(0, 0, 0);
    alliance_color = "blue";
    console.println("This is blue left awp");
    chassis.moveToPoint(42, 26, 2000, {.maxSpeed = 96});
    chassis.turnToHeading(-45, 500, {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE});
    chassis.moveToPoint(30, 37, 750, {.maxSpeed = 96}, false);
    pros::delay(1000);
    chassis.moveToPoint(45, 22, 500, {.forwards = false});
    chassis.moveToPoint(-10, 25, 1500, {.maxSpeed = 96});
    chassis.turnToHeading(-135, 750, {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE});
    chassis.moveToPoint(3, 36, 750, {.forwards = false}, false);
    pros::delay(1000);
    chassis.moveToPoint(-34, 0, 1500, {.maxSpeed = 96});
    chassis.turnToHeading(180, 750, {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE});
    chassis.moveToPoint(-34, -10, 500, {}, false);
    pros::delay(1000);
    chassis.moveToPoint(-33, 12, 750, {.forwards = false, .maxSpeed = 100}, false);
    pros::delay(1000);
}

void blue_left() {
    chassis.setPose(0, 0, 0);
    alliance_color = "blue";
    console.println("This is blue left ");
    intake_balls();
    chassis.moveToPoint(-3.5, 12, 750, {.minSpeed = 24, .earlyExitRange = 1});
    chassis.moveToPoint(-7, 30.25, 4000, {.maxSpeed = 24});
    chassis.turnToHeading(225, 500, {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE});
    chassis.moveToPoint(4, 37, 1000, {.forwards = false}, false);
    score_intake("mid");
    activate_wall_loading();
    pros::delay(2000);
    stop_all_intake_motors();
    chassis.moveToPoint(-31, 5, 1500, {.maxSpeed = 100});
    chassis.turnToHeading(180, 500, {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE});
    activate_intake(true);
    chassis.moveToPoint(-31, -8, 1000, {.maxSpeed = 64}, false);
    pros::delay(650);
    stop_intake();
    chassis.moveToPoint(-31, 21, 750, {.forwards = false, .maxSpeed = 112}, false);
    deactivate_wall_loading();
    score_intake("high");
    pros::delay(2000);
    stop_all_intake_motors();
    chassis.moveToPoint(-31, 11, 1000);
    
    chassis.turnToHeading(-90, 500);
    activate_right_descore();
    chassis.moveToPoint(-39, chassis.getPose().y, 1000);
    chassis.turnToHeading(0, 500, {});
}

void auton_skills() {
    //without rejection
    chassis.setPose(-62.125, -16.935, 90);
    chassis.moveToPoint(-46.379, -16.935, 750);
    chassis.turnToHeading(180, 500);
    chassis.moveToPoint(-46.379, -46.935, 1000, {.maxSpeed = 96});
    chassis.turnToHeading(270, 500);
    activate_wall_loading();
    activate_intake(true);
    chassis.moveToPoint(-60.125, -46.935, 500, {}, false);
    pros::delay(2000);
    chassis.moveToPoint(-53.487, -46.935, 500, {.forwards = false}, false);
    deactivate_wall_loading();
    stop_intake();
    chassis.turnToHeading(180, 500);
    chassis.moveToPoint(-53.487, -61.162, 500, {.maxSpeed = 96});
    chassis.turnToHeading(90, 500);
    chassis.moveToPoint(34.247, -61.162, 3000, {.maxSpeed = 96});
    chassis.turnToHeading(0, 500);
    chassis.moveToPoint(34.247, -46.935, 500, {.maxSpeed = 96});
    chassis.turnToHeading(90, 500);
    chassis.moveToPoint(30.125, -46.935, 500, {.forwards = false, .maxSpeed = 96}, false);
    score_intake("high");
    activate_wall_loading();
    pros::delay(3000);
    stop_scoring();

    chassis.setPose(30.125, -46.935, 90);
    chassis.moveToPoint(60.125, -46.935, 1000, {.maxSpeed = 96}, false);
    pros::delay(2000);
    chassis.moveToPoint(30.125, -46.935,1000, {.forwards = false, .maxSpeed = 96}, false);
    score_intake("high");
    deactivate_wall_loading();
    pros::delay(3000);
    stop_all_intake_motors();
    chassis.setPose(30.125, -46.935, 90);
    chassis.moveToPoint(34.247, -46.935, 500, {.maxSpeed = 96});
    chassis.turnToPoint(28.43, -34.164, 500);
    chassis.moveToPoint(28.43, -34.164, 750, {.minSpeed = 24, .earlyExitRange = 1});
    chassis.moveToPoint(23.595, -23.832, 2500, {.maxSpeed = 24});
    chassis.turnToPoint(-11.148, -23.832, 500);
    chassis.moveToPoint(-11.148, -23.832, 750, {.minSpeed = 24, .earlyExitRange = 1});
    chassis.moveToPoint(-23.595, -23.832, 2500, {.maxSpeed = 24});
    chassis.turnToPoint(-9.611, -9.848, 500);
    chassis.moveToPoint(-9.611, -9.848, 500, {}, true);
    score_intake("low");
    pros::delay(2000);
    stop_all_intake_motors();
    chassis.moveToPoint(-14.042, -4.279,500, {.forwards = false});
    chassis.turnToPoint(-54.598, 46.935, 500);
    chassis.moveToPoint(-54.598, 46.935, 2500, {.maxSpeed = 96}, false);
    activate_wall_loading();
    chassis.turnToHeading(-90, 500);
    activate_intake(true);
    chassis.moveToPoint(-60.125, 46.935, 500, {}, false);
    pros::delay(2000);
    stop_all_intake_motors();
    deactivate_wall_loading();
    chassis.turnToHeading(0, 500);
    chassis.moveToPoint(-54.598, 61.162, 500);
    chassis.turnToHeading(90, 500);
    chassis.moveToPoint(34.247, 61.162, 3000, {.maxSpeed = 96});
    chassis.turnToHeading(180, 750);
    chassis.moveToPoint(34.247, 46.935, 500, {.maxSpeed = 96});
    chassis.turnToHeading(90, 500);
    chassis.moveToPoint(30.125, 46.935, 500, {.forwards = false}, false);
    score_intake("high");
    activate_wall_loading();
    pros::delay(2000);
    stop_scoring();

    chassis.setPose(30.125, 46.935, 90);
    chassis.moveToPoint(60.125, 46.935, 1000, {.maxSpeed = 96}, false);
    pros::delay(2000);
    chassis.moveToPoint(30.125, 46.935, 1000, {.forwards = false}, false);
    score_intake("high");
    deactivate_wall_loading();
    pros::delay(2000);
    stop_all_intake_motors();

    chassis.setPose(30.125, 46.935, 90);
    chassis.moveToPoint(34.247, 46.935, 500);
    chassis.turnToPoint(28.418, 33.955, 500);
    chassis.moveToPoint(28.418, 33.955, 750, {.minSpeed = 24, .earlyExitRange = 1});
    chassis.moveToPoint(23.832, 23.832, 2500, {.maxSpeed = 24});
    chassis.turnToPoint(-11.584, 23.832, 500);
    chassis.moveToPoint(-11.584, 23.832, 1000, {.minSpeed = 24, .earlyExitRange = 1});
    chassis.moveToPoint(-23.595, 23.832, 2500, {.maxSpeed = 24});
    chassis.turnToHeading(-45, 500);
    chassis.moveToPoint(-9.611, 9.848, 750, {}, false);
    score_intake("mid");
    pros::delay(2000);
    chassis.moveToPoint(-14.042, 14.279, 500);
    chassis.turnToPoint(-38.232, 0, 750);
    chassis.turnToHeading(-90, 500);
    chassis.moveToPoint(-65, 0, 3000);
    
}

rd::Selector selector({
    {"Red Right AWP", red_right_awp},
    {"Red Left AWP", red_left_awp},
    {"Red Right", red_right},
    {"Red Left", red_left},
    {"Blue Right AWP", blue_right_awp},
    {"Blue Left AWP", blue_left_awp},
    {"Blue Right", blue_right},
    {"Blue Left", blue_left},
    {"Auton Skills", auton_skills}
});






/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

void initialize() {
    chassis.calibrate();
    team_image.focus();
    //pros::Task run_auto_rejector(auto_reject);
    //pros::Task run_stop_intake_stalling(stop_intake_stalling);

    /*pros::Task coordinateTask([&]()-> void {
        while(true) {
            console.println("color: " + detect_color());
            console.println("distance: " + std::to_string(color_sensor.get_proximity()));
            console.println("alliance color: " + alliance_color);
            console.println(std::to_string(detect_color() != alliance_color));
            pros::delay(50);
            console.clear();

        }
    }); */
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
    secret_image.focus();
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
    red_right();
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
            toggle_auto_reject();
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