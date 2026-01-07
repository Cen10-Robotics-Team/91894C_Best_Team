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
                              12.8011811, // 10 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
                              360, // drivetrain rpm is 360
                              2 // horizontal drift is 2 (for now)
);
// create an imu on port 10
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
pros::adi::Pneumatics intake_piston('b', false);
pros::adi::Pneumatics descore_piston('c', false);

pros::Optical color_sensor(10);
pros::Distance distance_sensor(9);

pros::Controller controller(pros::E_CONTROLLER_MASTER);

LV_IMG_DECLARE(Huskytech);
rd::Image team_image(&Huskytech, "Team Image");

rd::Console console;

bool enable_auto_reject = false;
std::string alliance_color = "";
bool scoring = false;

void activate_wall_loading() {
    intake_piston.extend();
}

void deactivate_wall_loading() {
    intake_piston.retract();
}

void activate_descore() {
    descore_piston.extend();
}

void deactivate_descore() {
    descore_piston.retract();
}

void activate_intake(bool direction){
    if(direction == true) {
        intake_motor_1.move(127);
        intake_motor_2.move(127);
        intake_motor_3.move(127);

    } else {
        intake_motor_1.move(-127);  
        intake_motor_2.move(-127);
        intake_motor_3.move(-127);
        
    }
}

void intake_balls() {
    intake_motor_1.move(127);
    intake_motor_2.move(127);
}

void stop_intake() {
    intake_motor_1.brake();
    intake_motor_2.brake();
    intake_motor_3.brake();
}

void stop_scoring() {
    scoring_motor.brake();
}

void stop_all_intake_motors() {
    stop_intake();
    stop_scoring();
}

void score_intake(std::string goal) {
    if(goal == "low") {
        stop_scoring();
        activate_intake(false);
        scoring = true;
    } else if (goal == "mid") {
        scoring_piston.retract();
        scoring_motor.move(127);
        pros::delay(200);
        intake_motor_3.move(127);
        pros::delay(200);
        intake_motor_2.move(127);
        pros::delay(200);
        intake_motor_1.move(127);
        scoring = true;
    } else if (goal == "high") {
        scoring_piston.extend();
        scoring_motor.move(-127);
        pros::delay(200);
        intake_motor_3.move(127);
        pros::delay(200);
        intake_motor_2.move(127);
        pros::delay(200);
        intake_motor_1.move(127);
        scoring = true;
    }
}

void reject_intake() {
    score_intake("mid");
    pros::delay(200);
    stop_intake();
}

std::string detect_color() {
    int rgb_value = color_sensor.get_hue();
    if(rgb_value >= 160 && rgb_value <= 230) {
        return "blue";
    } else if (rgb_value >= 350 || rgb_value <= 20) {
        return "red";
    } else {
        return "";
    }
}

void auto_reject() {
    while(true) {
        while (enable_auto_reject) {
            std::string block_color = detect_color();
            if(block_color != alliance_color && distance_sensor.get() < 50) {
                reject_intake();
            }
        }
    }
}

void toggle_auto_reject() {
    enable_auto_reject = !enable_auto_reject;
}

void red_right_awp() {
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
    alliance_color = "red";
    console.println("This is red right");
    chassis.moveToPoint(6.5, 30, 750);
    chassis.turnToHeading(-45, 500, {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE});
    chassis.moveToPoint(-3, 38, 1000, {}, false);
    pros::delay(1000);
    chassis.moveToPoint(32, 0, 1250, {.forwards = false, .maxSpeed = 112});
    chassis.turnToHeading(180, 500, {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE});
    chassis.moveToPoint(32, -7, 1000, {}, false);
    pros::delay(1000);
    chassis.moveToPose(33, 15, 180, 750, {.forwards = false, .maxSpeed = 100}, false);
    pros::delay(1000);
    chassis.swingToHeading(0, DriveSide::RIGHT, 1500, {.direction = lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 64, .minSpeed = 64, .earlyExitRange = 5});
    chassis.moveToPoint(17.5, 32, 750);
}

void red_left_awp() {
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
    alliance_color = "red";
    console.println("This is red left");
    chassis.moveToPoint(-6.5, 30, 750);
    chassis.turnToHeading(-135, 500, {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE});
    chassis.moveToPoint(3, 38, 1000, {.forwards = false}, false);
    pros::delay(1000);
    chassis.moveToPoint(-32, 0, 1250, {.maxSpeed = 112});
    chassis.turnToHeading(180, 500, {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE});
    chassis.moveToPoint(-32, -7, 1000, {}, false);
    pros::delay(1000);
    chassis.moveToPose(-33, 15, 180, 750, {.forwards = false, .maxSpeed = 100}, false);
    pros::delay(1000);
    chassis.swingToHeading(0, DriveSide::LEFT, 1500, {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 64, .minSpeed = 64, .earlyExitRange = 5});
    chassis.moveToPoint(-17.5, 32, 750); 
}

void blue_right_awp() {
    alliance_color = "blue";
    console.println("This is blue right awp");
    intake_balls();
    chassis.turnToPoint(-30, 18.5, 500);
    chassis.moveToPoint(-30, 18.5, 1500, {.maxSpeed = 96, .minSpeed = 32, .earlyExitRange = 1});
    chassis.moveToPoint(-44, 29, 3000, {.maxSpeed = 32});
    chassis.turnToHeading(-135, 500, {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE}, false);
    stop_intake();
    chassis.moveToPoint(-30, 37, 750, {.forwards = false, .maxSpeed = 96}, false);
    score_intake("mid");
    pros::delay(1000);
    stop_intake();
    intake_balls();
    chassis.moveToPoint(-33, 34, 500);
    chassis.turnToPoint(-6, 29, 500);
    chassis.moveToPoint(-6, 29, 1000, {.minSpeed = 32, .earlyExitRange = 1});
    chassis.moveToPoint(6.5, 26, 3000, {.maxSpeed = 32});
    chassis.turnToPoint(-4, 34, 750);
    chassis.moveToPoint(-4, 34, 750, {}, false);
    score_intake("low");
    pros::delay(1000);
    chassis.moveToPoint(31, 0, 1500, {.forwards = false, .maxSpeed = 96});
    chassis.turnToHeading(180, 750, {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE}, false);
    activate_wall_loading();
    chassis.moveToPoint(31, -7, 750, {});
    activate_intake(true);
    pros::delay(1000);
    stop_intake();
    chassis.moveToPoint(32, 19, 750, {.forwards = false, .maxSpeed = 100}, false);
    score_intake("high");
}

void blue_right() {
    alliance_color = "blue";
    console.println("This is blue right ");
    intake_balls();
    chassis.moveToPoint(2.5, 12, 750, {.minSpeed = 24, .earlyExitRange = 1});
    chassis.moveToPoint(6.5, 30, 3500, {.maxSpeed = 24});
    chassis.turnToHeading(-45, 500, {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE});
    chassis.moveToPoint(-3, 38, 1000, {}, false);
    score_intake("low");
    pros::delay(1000);
    stop_intake();
    chassis.moveToPoint(33, 0, 1250, {.forwards = false, .maxSpeed = 112});
    chassis.turnToHeading(180, 500, {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE}, false);
    activate_wall_loading();
    chassis.moveToPoint(33, -7, 1000, {});
    activate_intake(true);
    pros::delay(1000);
    stop_intake();
    chassis.moveToPoint(34, 20, 750, {.forwards = false, .maxSpeed = 100}, false);
    score_intake("high");
    pros::delay(1000);
    stop_all_intake_motors();
    chassis.moveToPoint(33, 16, 500);
    
    chassis.turnToHeading(-90, 500);
    chassis.moveToPoint(21, chassis.getPose().y, 750);
    chassis.turnToHeading(0, 500, {}, false);
    activate_descore();
    chassis.moveToPoint(21, 30, 750);
}

void blue_left_awp() {
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
    alliance_color = "blue";
    console.println("This is blue left");
    chassis.moveToPoint(-6.5, 30, 750);
    chassis.turnToHeading(-135, 500, {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE});
    chassis.moveToPoint(3, 38, 1000, {.forwards = false}, false);
    pros::delay(1000);
    chassis.moveToPoint(-32, 0, 1250, {.maxSpeed = 112});
    chassis.turnToHeading(180, 500, {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE});
    chassis.moveToPoint(-32, -7, 1000, {}, false);
    pros::delay(1000);
    chassis.moveToPose(-33, 15, 180, 750, {.forwards = false, .maxSpeed = 100}, false);
    pros::delay(1000);
    chassis.swingToHeading(0, DriveSide::LEFT, 1500, {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 64, .minSpeed = 64, .earlyExitRange = 5});
    chassis.moveToPoint(-17.5, 32, 750); 
}

void auton_skills() {

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
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

void initialize() {
    chassis.calibrate();
    selector.focus();
    pros::Task run_auto_rejector(auto_reject);

    /*pros::Task coordinateTask([&]()-> void {
        while(true) {
            pros::lcd::print(0, " %f", chassis.getPose().x);
            pros::lcd::print(1, " %f", chassis.getPose().y);
            pros::lcd::print(2, " %f", chassis.getPose().theta);

            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            pros::delay(50);
        }
    }); */
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

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
    // set position to 0, 0, heading:0
    chassis.setPose(0, 0, 0);
    blue_right_awp();
    //selector.run_auton();

    //auton skills
    /*chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0, 35, 1500, {.maxSpeed = 80});
    chassis.turnToHeading(90, 750);
    chassis.moveToPoint(10, 35, 1500, {});
    pros::delay(2000);
    chassis.moveToPoint(7, 35, 1000, {.forwards = false});
    chassis.turnToHeading(180, 750);
    chassis.moveToPoint(7, 15, 1500, {.maxSpeed = 80});
    chassis.turnToHeading(-90, 750);
    chassis.moveToPoint(-88, 15, 3000, {.maxSpeed = 80});
    chassis.turnToHeading(0, 750);
    chassis.moveToPoint(-88, 35, 1500, {.maxSpeed = 80});
    chassis.turnToHeading(-90, 750);
    chassis.moveToPoint(-80, 35, 1500, {.forwards = false, .maxSpeed = 96}, false);
    pros::delay(2000);

    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0, 28, 1500, {}, false);
    pros::delay(2000);
    chassis.moveToPoint(0, 0, 1500, {.forwards = false}, false);
    pros::delay(2000);
    chassis.moveToPoint(0, 18, 1000, {.maxSpeed = 96});
    chassis.turnToPoint(-26.5, -8.5, 750);
    chassis.moveToPoint(-26.5, -8.5, 1500, {.maxSpeed = 96});
    chassis.turnToPoint(-29, -34, 750);
    chassis.moveToPoint(-29, -34, 1500, {.maxSpeed = 96});
    chassis.turnToHeading(90, 750);
    chassis.moveToPoint(-9, -34, 1500);
    chassis.moveToPoint(-29, -34, 1500, {.forwards = false});
    chassis.turnToPoint(-24.8, -57.4, 750);
    chassis.moveToPoint(-24.8, -57.4, 1500, {.maxSpeed = 96});
    chassis.turnToPoint(-72, -57.4, 750);
    chassis.moveToPoint(-72, -57.4, 1500, {.maxSpeed = 96});
    chassis.turnToHeading(225, 750);
    chassis.moveToPoint(-93.4, -46.3, 1500, {.forwards = false}, false);
    pros::delay(2000);
    chassis.moveToPoint(-129.5, -89.1, 1500, {.maxSpeed = 96});
    chassis.turnToHeading(180, 750);
    chassis.moveToPoint(-129.5, -94.5, 1500, {.maxSpeed = 96});
    chassis.moveToPoint(-129.5, -66.5, 1500, {.forwards = false, .maxSpeed = 96});
    */
    /*
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0, 5, 1000, {.maxSpeed = 96});
    chassis.turnToHeading(-90, 750);
    chassis.moveToPoint(-20, 5, 1500, {.maxSpeed = 80});
    chassis.turnToHeading(180, 750);
    chassis.moveToPoint(-20, -70, 3000, {.maxSpeed = 80});
    chassis.turnToHeading(90, 750);
    chassis.moveToPoint(0, -70, 1500, {.maxSpeed = 96});
    chassis.turnToHeading(180, 750);
    chassis.moveToPoint(0, -62, 1000, {.forwards = false}, false);
    pros::delay(2000);

    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0, 28, 1500, {}, false);
    pros::delay(2000);
    chassis.moveToPoint(0, 0, 1500, {.forwards = false}, false);
    pros::delay(2000);
    chassis.moveToPoint(0, 18, 1000, {.maxSpeed = 96});
    chassis.turnToPoint(26.5, -8.5, 750);
    chassis.moveToPoint(26.5, -8.5, 1500, {.maxSpeed = 96});
    chassis.turnToPoint(29, -34, 750);
    chassis.moveToPoint(29, -34, 1500, {.maxSpeed = 96});
    chassis.turnToHeading(-90, 750);
    chassis.moveToPoint(20, -34, 1500, {.maxSpeed = 96});
    chassis.moveToPoint(29, -34, 1500, {.forwards = false, .maxSpeed = 96});
    chassis.turnToPoint(26.5, -8.5, 750);
    chassis.moveToPoint(26.5, -8.5, 1500, {.maxSpeed = 96});
    chassis.turnToHeading(135, 750);
    chassis.moveToPoint(38.7, -19.6, 1500);
    */

    /*
    chassis.setPose(-46.5, -15.3, 180);
    chassis.moveToPoint(-46.5, -50.5, 1500, {.maxSpeed = 80});
    chassis.turnToHeading(270, 750);
    chassis.moveToPoint(-57, -51.5, 1000, {}, false);
    pros::delay(2000);
    chassis.moveToPoint(-53.9, -50.5, 1000, {.forwards = false});
    chassis.turnToHeading(180, 750);
    chassis.moveToPoint(-55, -63, 1000, {.maxSpeed = 80});
    chassis.turnToHeading(90, 750);
    
    chassis.moveToPoint(40, -63, 3000, {.maxSpeed = 80});
    
    chassis.turnToHeading(0, 750);
    chassis.moveToPoint(41.5, -49, 1000, {.maxSpeed = 80});
    chassis.turnToHeading(90, 750);
    
    chassis.moveToPoint(34, -49, 1000, {.forwards = false, .maxSpeed = 96}, false);
    pros::delay(2000);
    chassis.moveToPoint(62, -48, 1500, {.maxSpeed = 96}, false);
    pros::delay(2000);
    chassis.moveToPoint(34, -49, 1500, {.forwards = false, .maxSpeed = 96}, false);
    pros::delay(2000);

    chassis.setPose(34, -49, 90);
    chassis.moveToPoint(52, -49, 1000, {.maxSpeed = 96});
    
    chassis.turnToPoint(26.5, -21.5, 750);
    chassis.moveToPoint(26.5, -21.5, 1500, {.maxSpeed = 96});
    
    chassis.turnToPoint(0, -20, 750);
    chassis.moveToPoint(0, -20, 1500, {.maxSpeed = 96});
    chassis.turnToHeading(180, 750);
    chassis.moveToPoint(0, -40, 1500);
    chassis.moveToPoint(0, -20, 1500, {.forwards = false});

    chassis.turnToPoint(-23.4, -24.2, 750);
    chassis.moveToPoint(-23.4, -24.2, 1500, {.maxSpeed = 96});
    chassis.turnToPoint(-23.4, 23.6, 750);
    chassis.moveToPoint(-23.4, 23.6, 1500, {.maxSpeed = 96});
    chassis.turnToHeading(-45, 500);
    chassis.moveToPoint(-12.3, 11.4, 1500, {.forwards = false}, false);
    pros::delay(2000);

    chassis.moveToPoint(-55.1, 47.5, 1500, {.maxSpeed = 96});
    chassis.turnToHeading(270, 750);
    chassis.moveToPoint(-60.5, 47.5, 1500, {.maxSpeed = 96});
    pros::delay(2000);
    chassis.moveToPoint(-55.1, 47.5, 1500, {.forwards = false, .maxSpeed = 96});
    chassis.turnToHeading(0, 750);
    */
    /*
    chassis.moveToPoint(-55.1, 57.2, 750);
    chassis.turnToHeading(90, 500);
    chassis.moveToPoint(34.3, 57.2, 750);
    chassis.turnToHeading(180, 500);
    chassis.moveToPoint(34.3, 47.7, 750);
    chassis.turnToHeading(90, 500);
    chassis.moveToPoint(32, 47.7, 750, {.forwards = false});
    chassis.moveToPoint(59.1, 47.7, 750);
    chassis.moveToPoint(32, 57.2, 750, {.forwards = false});
    chassis.moveToPoint(35, 57.2, 750);
    //small shifting
    chassis.turnToPoint(24, 24, 750);
    chassis.moveToPoint(24, 24, 750);
    chassis.moveToPoint(0, 33.0, 750);
    chassis.turnToHeading(0, 500);
    chassis.moveToPoint(0, 43, 750);
    chassis.moveToPoint(19.4, 18.1, 750, {.forwards = false});
    chassis.turnToHeading(225, 500);
    chassis.moveToPoint(8.4, 7.4, 750);
    */
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

        if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
            toggle_auto_reject();
        }

        // delay to save resources
        pros::delay(20); 
    }
}