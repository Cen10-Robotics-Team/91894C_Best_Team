#pragma once
#include "api.h"
#include "lemlib/api.hpp" // IWYU pragma: keep

// Changed the left motors to be all the same direction and right motors in the same direction as well. Be happy. -normalperson543
inline pros::MotorGroup left_motors({-1, -2, -3}, pros::MotorGearset::blue); // left motors on ports 1, 2, 3 
inline pros::MotorGroup right_motors({4, 5, 6}, pros::MotorGearset::blue); // right motors on ports 4, 5, 6
// drivetrain settings
inline lemlib::Drivetrain drivetrain(&left_motors, // left motor group
                              &right_motors, // right motor group
                              11.25, // 11.3125 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
                              450, // drivetrain rpm is 360
                              2 // horizontal drift is 2 (for now)
);

// input curve for throttle input during driver control
inline lemlib::ExpoDriveCurve throttle_curve(15, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.003 // expo curve gain
);

// input curve for steer input during driver control
inline lemlib::ExpoDriveCurve steer_curve(15, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.003 // expo curve gain
);

//inertial sensor on port 11
inline pros::Imu imu(11);

// create v5 rotation sensors on ports 12 & 13
inline pros::Rotation rotation_sensor_horizontal(-13);
inline pros::Rotation rotation_sensor_vertical(-12);

// horizontal tracking wheel
inline lemlib::TrackingWheel horizontal_tracking_wheel(&rotation_sensor_horizontal, lemlib::Omniwheel::NEW_2, -0.51);

// vertical tracking wheel
inline lemlib::TrackingWheel vertical_tracking_wheel(&rotation_sensor_vertical, ::lemlib::Omniwheel::NEW_2, 0.06);

// Odom Sensors
inline lemlib::OdomSensors sensors(&vertical_tracking_wheel, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            &horizontal_tracking_wheel, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// lateral PID controller
inline lemlib::ControllerSettings lateral_controller(14.1, // proportional gain (kP)
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
inline lemlib::ControllerSettings angular_controller(1.5, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in inches
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);
//chassis definition
inline lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors,
                        &throttle_curve,
                        &steer_curve // odometry sensors
);