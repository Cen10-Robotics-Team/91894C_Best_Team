#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "liblvgl/widgets/lv_img.h"
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

pros::Controller controller(pros::E_CONTROLLER_MASTER);

LV_IMG_DECLARE(Huskytech);
rd::Image team_image(&Huskytech, "Team Image");

rd::Console console;

void red_right_awp() {
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
    console.println("This is blue right awp");
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

void blue_right() {
    console.println("This is blue right ");
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

void blue_left_awp() {
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

rd::Selector selector({
    {"Red Right AWP", red_right_awp},
    {"Red Left AWP", red_left_awp},
    {"Red Right", red_right},
    {"Red Left", red_left},
    {"Blue Right AWP", blue_right_awp},
    {"Blue Left AWP", blue_left_awp},
    {"Blue Right", blue_right},
    {"Blue Left", blue_left},

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
void measureSteadyStateError() {
    std::vector<int> angles;
    double totalAverageError = 0;
    for (int a = 10; a <= 180; a += 10) angles.push_back(a);

    for (int angle : angles) {
        double sumError = 0;

        for (int trial = 1; trial <= 3; trial++) {

            // Command a turn
            chassis.turnToHeading(angle, 3000); // 3 sec timeout

            // Wait for settle (important!)
            pros::delay(500);  // Give PID time to finish

            // Read the actual angle
            double actual = imu.get_rotation();

            // Normalize negative wrap
            if (actual < 0) actual += 360;

            // Calculate steady-state error
            double error = actual - angle;

            sumError += fabs(error);

            pros::lcd::print(0, "Angle: %d  Trial: %d", angle, trial);
            pros::lcd::print(1, "Actual: %.2f  Err: %.2f", actual, error);

            // Wait before next trial
            pros::delay(600);
        }

        double avg = sumError / 3.0;

        pros::lcd::print(2, "AVERAGE ERROR for %d deg: %.2f deg", angle, avg);

        // Print to terminal too
        printf("Angle %d | Avg steady-state error: %.2f deg\n", angle, avg);

        totalAverageError += avg;

        pros::delay(1500);
    }
    pros::lcd::print(4, "Average wideup range: %.2f deg", totalAverageError / 18);

    printf("=== DONE MEASURING STEADY-STATE ERROR ===\n");
}

void initialize() {
    chassis.calibrate();
    team_image.focus();

    /*pros::Task coordinateTask([&]()-> void {
        while(true) {
            pros::lcd::print(0, "X: %f", chassis.getPose().x);
            pros::lcd::print(1, "Y: %f", chassis.getPose().y);
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta);

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
    // set position to x:0, y:0, heading:0
    chassis.setPose(0, 0, 0);
    selector.run_auton();

    //auton skills
    void autonomous() {

chassis.setPose(x:-46.5, y:-15.3, theta:180);
chassis.moveToPoint(x:-47.1, y:-47.4, timeout:750);
chassis.turnToHeading(theta:90, timeout:500);
chassis.moveToPoint(x:-60.1, y:-46.9, timeout:750);
chassis.turnToHeading(theta:180, timeout:500);
chassis.moveToPoint(x:-53.9, y:-46.9, timeout:750);
chassis.turnToHeading(theta:90, timeout:500);
chassis.moveToPoint(x:-53.3, y:-55.9, timeout:750);
chassis.turnToHeading(theta:-90. timeout:500);
chassis.moveToPoint(x:34.1, y:-55.6);
chassis.turnToHeading(theta:-90, timeout:500);
chassis.moveToPoint(x:30.4, y:-47.2, timeout:750);
chassis.turnToHeading(theta:90, timeout:500);
chassis.moveToPoint(x:58.5, y:-47.7, timeout:750);
chassis.turnToHeading(theta:180, timeout:500);
chassis.moveToPoint(x:30.4, y:-47.2, timeout:750);
//small shifting
chassis.turnToHeading(theta:45, timeout:500);
chassis.moveToPoint(x:23.8, y:-24.0, timeout:750);
chassis.turnToHeading(theta:-90, timeout:500);
chassis.moveToPoint(x:-0.1, y:-37.7, timeout:750);
//small shifting
chassis.turnToHeading(theta:45, timeout:500);
chassis.moveToPoint(x:-23.4, y:-24.2, timeout:750);
chassis.turnToHeading(theta:45, timeout:500);
chassis.moveToPoint(x:-23.6, y:23.6, timeout:750);
chassis.turnToHeading(theta:135, timeout:500);
chassis.moveToPoint(x:-12.3, y:11.4, timeout:750);
chassis.turnToHeading(theta:180, timeout:500);
chassis.moveToPoint(x:-23.6, y:23.6, timeout:750);
chassis.turnToHeading(theta:-45, timeout:500);
chassis.moveToPoint(x:-55.1, y:46.1, timeout:750);
chassis.turnToHeading(theta:-45, timeout:500);
chassis.moveToPoint(x:-60.5, y:46.1, timeout:750);
chassis.turnToHeading(theta:180, timeout:500);
chassis.moveToPoint(x:-55.1, y:46.1, timeout:750);
chassis.turnToHeading(theta:-90, timeout:500);
chassis.moveToPoint(x:-56.9, y:57.2, timeout:750);
chassis.turnToHeading(theta:90, timeout:500);
chassis.moveToPoint(x:34.3, y:57.2, timeout:750);
chassis.turnToHeading(theta:90, timeout:500);
chassis.moveToPoint(x:33.8, y:57.2, timeout:750);
chassis.turnToHeading(theta:-90, timeout:500);
chassis.moveToPoint(x:59.1, y:46.6, timeout:750);
chassis.turnToHeading(theta:180, timeout:500);
chassis.moveToPoint(x:33.8, y:57.2, timeout:750);
//small shifting
chassis.turnToHeading(theta:-45, timeout:500);
chassis.moveToPoint(x:23.8, y:24.2, timeout:750);
chassis.turnToHeading(theta:90, timeout:500);
chassis.moveToPoint(x:0.1, y:33.0, timeout:750);
chassis.turnToHeading(theta:45, timeout:500);
chassis.moveToPoint(x:0.1, y:40.5, timeout:750);
chassis.turnToHeading(theta:135, timeout:500);
chassis.moveToPoint(x:19.4, y:18.1, timeout:750);
chassis.turnToHeading(theta:90, timeout:500);
chassis.moveToPoint(x:8.4, y:7.4, timeout:750);
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

        

        // delay to save resources
        pros::delay(20); 
    }
}