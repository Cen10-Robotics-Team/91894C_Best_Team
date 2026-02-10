#include "lemlib/chassis/chassis.hpp"
#include "main.h"

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

void blue_right() {
    //right_long_plus_mid path
    chassis.setPose(-61.625, -16.185, 90);
    alliance_color = "blue";
    console.println("This is blue right long + mid ");
    activate_intake(true);

    //move to balls and intake
    chassis.moveToPoint(-34, -16.185, 1000, {}, false);
    chassis.turnToPoint(-18, -26, 500);
    chassis.moveToPoint(-18, -26, 2000, {.maxSpeed = 36});

    //go to low goal
    chassis.turnToPoint(-11, -17, 750);
    chassis.moveToPoint(-11, -17, 750, {}, false);
    score_intake("low");
    pros::delay(1500);
    stop_all_intake_motors();

    //move to wall loader and intake
    chassis.turnToPoint(-45, -50, 500, {.forwards = false});
    chassis.moveToPoint(-45, -50, 1500, {.forwards = false}, false);
    activate_wall_loading();
    chassis.turnToHeading(270, 500);
    activate_intake(true);
    chassis.moveToPoint(-56, -50, 500, {.maxSpeed = 70}, false);

    chassis.moveToPoint(-52, -50, 150, {.forwards = false, .maxSpeed = 64});
    chassis.moveToPoint(-56, -50, 150);
    chassis.moveToPoint(-52, -50, 150, {.forwards = false, .maxSpeed = 64});
    chassis.moveToPoint(-56, -50, 150);
    chassis.moveToPoint(-52, -50, 150, {.forwards = false, .maxSpeed = 64});
    chassis.moveToPoint(-56, -50, 150);
    
    
    /*chassis.turnToHeading(275, 100);
    chassis.turnToHeading(265, 100);
    chassis.turnToHeading(275, 100);
    chassis.turnToHeading(265, 100);
    chassis.turnToHeading(275, 100);
    chassis.turnToHeading(270, 100);*/
    //pros::delay(600);

    //move to high goal and score
    chassis.moveToPoint(-23, -50, 1500, {.forwards = false}, false);
    deactivate_wall_loading();
    score_intake("high");
    pros::delay(1500);

    //reset position and move to descore
    chassis.setPose(-28, -46, 270);
    stop_all_intake_motors();
    chassis.moveToPoint(-42, -34, 1100);
    chassis.turnToHeading(90, 500, {});
    activate_right_descore();
    chassis.moveToPoint(-13, -33, 3000);
    pros::delay(250);
    deactivate_right_descore();

}

void blue_right_long() {
    chassis.setPose(-62.125, -16.935, 90);
    alliance_color = "blue";
    console.println("This is blue right long");
    activate_intake(true);

    //move to balls and intake
    chassis.moveToPoint(-34, -16.935, 1000, {}, false);
    chassis.turnToPoint(-20, -24, 500);
    chassis.moveToPoint(-20, -24, 2500, {.maxSpeed = 24});

    //move to wall loader and intake
    chassis.turnToPoint(-45, -50, 500);
    chassis.moveToPoint(-45, -50, 1500, {}, false);
    activate_wall_loading();
    chassis.turnToHeading(270, 500);
    activate_intake(true);
    chassis.moveToPoint(-56, -50, 500, {.maxSpeed = 70}, false);

    chassis.moveToPoint(-52, -50, 150, {.forwards = false, .maxSpeed = 64});
    chassis.moveToPoint(-56, -50, 150);
    chassis.moveToPoint(-52, -50, 150, {.forwards = false, .maxSpeed = 64});
    chassis.moveToPoint(-56, -50, 150);
    chassis.moveToPoint(-52, -50, 150, {.forwards = false, .maxSpeed = 64});
    chassis.moveToPoint(-56, -50, 150);
    chassis.moveToPoint(-52, -50, 150, {.forwards = false, .maxSpeed = 64});
    chassis.moveToPoint(-56, -50, 150);
    chassis.moveToPoint(-52, -50, 150, {.forwards = false, .maxSpeed = 64});
    chassis.moveToPoint(-56, -50, 150);

    //move to high goal and score
    chassis.moveToPoint(-23, -51, 1500, {.forwards = false}, false);
    deactivate_wall_loading();
    score_intake("high");
    pros::delay(2500);

    //reset position and move to descore
    chassis.setPose(-28, -46, 270);
    stop_all_intake_motors();
    chassis.moveToPoint(-42, -34, 1100, {});
    chassis.turnToHeading(90, 500, {});
    activate_right_descore();
    chassis.moveToPoint(-13, -33, 3000);
    pros::delay(250);
    deactivate_right_descore();
    scoring_piston.retract();
}

void blue_left() {
    //right_long_plus_mid path
    chassis.setPose(-62.125, 16.935, 90);
    alliance_color = "blue";
    console.println("This is blue left long + mid ");
    activate_intake(true);
    //move to balls and intake
    chassis.moveToPoint(-34, 16.935, 1000, {.maxSpeed = 96}, false);
    chassis.turnToPoint(-20, 25, 500);
    chassis.moveToPoint(-20, 25, 2300, {.maxSpeed = 24});

    //go to mid goal
    chassis.turnToHeading(-45, 500);
    chassis.moveToPoint(-10, 14, 750, {.forwards = false, .maxSpeed = 80}, false);
    score_intake("mid");
    pros::delay(1500);
    stop_all_intake_motors();

    //move to wall loader and intake
    chassis.moveToPoint(-45, 50, 1400, { .maxSpeed = 96}, false);
    activate_wall_loading();
    chassis.turnToHeading(270, 500);
    activate_intake(true);
    chassis.moveToPoint(-54, 50, 1100, {.maxSpeed = 80}, false);
    
    /*chassis.turnToHeading(265, 100);
    chassis.turnToHeading(275, 100);
    chassis.turnToHeading(265, 100);
    chassis.turnToHeading(275, 100);
    chassis.turnToHeading(265, 100);
    chassis.turnToHeading(275, 100);
    chassis.turnToHeading(265, 100);
    chassis.turnToHeading(270, 100);*/
    pros::delay(600);

    //move to high goal and score
    chassis.moveToPoint(-24, 51, 1400, {.forwards = false, .maxSpeed = 90}, false);
    deactivate_wall_loading();
    score_intake("high");
    pros::delay(1400);

    //reset position and move to descore
    chassis.setPose(-28, 46, 270);
    stop_all_intake_motors();
    chassis.moveToPoint(-42, 59, 1100, {.maxSpeed = 96});
    chassis.turnToHeading(90, 500, {});
    activate_right_descore();
    chassis.moveToPoint(-13, 60, 10000);
    pros::delay(250);
    deactivate_right_descore();
}

void blue_left_long() {
    //right_long_plus_mid path
    chassis.setPose(-62.125, 16.935, 90);
    alliance_color = "blue";
    console.println("This is blue left long");
    activate_intake(true);
    //move to balls and intake
    chassis.moveToPoint(-34, 16.935, 1500, {.maxSpeed = 96}, false);
    chassis.turnToPoint(-20, 25, 500);
    chassis.moveToPoint(-20, 25, 2500, {.maxSpeed = 24});

    //move to wall loader and intake
    chassis.turnToPoint(-45, 48, 500);
    chassis.moveToPoint(-45, 48, 1500, { .maxSpeed = 96}, false);
    activate_wall_loading();
    chassis.turnToHeading(270, 500);
    activate_intake(true);
    chassis.moveToPoint(-54, 48, 1100, {.maxSpeed = 80}, false);
    /*chassis.turnToHeading(265, 100);
    chassis.turnToHeading(275, 100);
    chassis.turnToHeading(265, 100);
    chassis.turnToHeading(275, 100);
    chassis.turnToHeading(265, 100);
    chassis.turnToHeading(275, 100);
    chassis.turnToHeading(265, 100);
    chassis.turnToHeading(275, 100);
    chassis.turnToHeading(265, 100);
    chassis.turnToHeading(275, 100);
    chassis.turnToHeading(270, 100);*/
    pros::delay(600);

    //move to high goal and score
    chassis.moveToPoint(-24, 49, 1500, {.forwards = false, .maxSpeed = 90}, false);
    deactivate_wall_loading();
    score_intake("high");
    pros::delay(2500);

    //reset position and move to descore
    chassis.setPose(-28, 46, 270);
    stop_all_intake_motors();
    chassis.moveToPoint(-42, 59, 1500, {.maxSpeed = 96});
    chassis.turnToHeading(90, 500, {});
    activate_right_descore();
    chassis.moveToPoint(-13, 59, 10000);
    pros::delay(250);
    deactivate_right_descore();
}

void auton_skills() {
    chassis.setPose(-62.125, -16.935, 90);
    chassis.moveToPoint(-46.379, -16.935, 750);
    chassis.turnToHeading(180, 500);
    chassis.moveToPoint(-46.379, -50, 1000, {.maxSpeed = 96});
    chassis.turnToHeading(270, 500);
    activate_wall_loading();
    activate_intake(true);
    chassis.moveToPoint(-56, -50, 500, {}, false);

    chassis.moveToPoint(-54, -50, 150);
    chassis.moveToPoint(-56, -50, 150);
    chassis.moveToPoint(-54, -50, 150);
    chassis.moveToPoint(-56, -50, 150);
    chassis.moveToPoint(-54, -50, 150);
    chassis.moveToPoint(-56, -50, 150);
    chassis.moveToPoint(-54, -50, 150);
    chassis.moveToPoint(-56, -50, 150);
    chassis.moveToPoint(-54, -50, 150);
    chassis.moveToPoint(-56, -50, 150);
    chassis.moveToPoint(-54, -50, 150);
    chassis.moveToPoint(-56, -50, 150);
    chassis.moveToPoint(-54, -50, 150);
    chassis.moveToPoint(-56, -50, 150);
    chassis.moveToPoint(-54, -50, 150);

    chassis.moveToPoint(-40, -50, 1000, {.forwards = false}, false);
    deactivate_wall_loading();
    stop_intake();
    chassis.turnToHeading(180, 500);
    chassis.moveToPoint(-53, -61, 500, {.maxSpeed = 96});
    chassis.turnToHeading(90, 500);
    chassis.moveToPoint(50, -61, 3000, {.maxSpeed = 96});
    chassis.turnToHeading(0, 500);
    chassis.moveToPoint(50, -50, 500, {.maxSpeed = 96});
    chassis.turnToHeading(90, 500);
    chassis.moveToPoint(40, -50, 1000, {.forwards = false, .maxSpeed = 96}, false);
    score_intake("high");
    activate_wall_loading();
    pros::delay(4000);
    stop_scoring();

    chassis.setPose(30, -50, 90);
    chassis.moveToPoint(54, -50, 1000, {.maxSpeed = 96}, false);

    chassis.moveToPoint(52, -50, 150);
    chassis.moveToPoint(54, -50, 150);
    chassis.moveToPoint(52, -50, 150);
    chassis.moveToPoint(54, -50, 150);
    chassis.moveToPoint(52, -50, 150);
    chassis.moveToPoint(52, -50, 150);
    chassis.moveToPoint(54, -50, 150);
    chassis.moveToPoint(52, -50, 150);
    chassis.moveToPoint(54, -50, 150);
    chassis.moveToPoint(52, -50, 150);
    chassis.moveToPoint(54, -50, 150);
    chassis.moveToPoint(52, -50, 150);
    chassis.moveToPoint(54, -50, 150);
    chassis.moveToPoint(52, -50, 150);
    chassis.moveToPoint(54, -50, 150);

    chassis.moveToPoint(28, -50,1500, {.forwards = false, .maxSpeed = 96}, false);
    score_intake("high");
    deactivate_wall_loading();
    pros::delay(4000);
    stop_all_intake_motors();

    chassis.setPose(30, -46, 90);
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

void do_nothing_auton() {
    chassis.setPose(0,0,0);
    chassis.moveToPoint(0, 4, 2000);
}

void pid_auton() {
    chassis.setPose(0,0,0); 
    chassis.moveToPoint(0,24,100000);   
};   