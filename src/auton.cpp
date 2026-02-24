#include "lemlib/chassis/chassis.hpp"
#include "main.h"


void right_awp_r4_auton() {
    chassis.setPose(-61.625, -16.185, 90);

    //move to right wall loader
    chassis.moveToPoint(-47, -16.185, 750);
    chassis.turnToHeading(180, 500);
    chassis.moveToPoint(-47, -49, 1000, {}, false);
    activate_wall_loading();
    chassis.turnToHeading(270, 500, {}, false);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);

    //intake from wall loader
    activate_intake(true);
    chassis.moveToPoint(-60, -49, 500, {.maxSpeed = 64}, false);
    pros::delay(150);
    chassis.moveToPoint(-61, -49, 100, {}, false);
    pros::delay(150);
    chassis.moveToPoint(-61, -49, 100);

    //score right high goal
    chassis.moveToPoint(-26, -49, 1000, {.forwards = false}, false);
    deactivate_wall_loading();
    score_intake("high");
    pros::delay(1000);
    stop_all_intake_motors();
    chassis.moveToPoint(-25, -49, 100, {}, false);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);

    //reset position and intake low goal blocks
    chassis.setPose(-28, -47, 270);
    chassis.moveToPoint(-38, -47, 750);
    chassis.turnToPoint(-26, -33, 500, {}, false);
    activate_intake(true);
    chassis.moveToPoint(-26, -33, 750, {.minSpeed = 32, .earlyExitRange = 1});
    chassis.moveToPoint(-20, -22, 1500, {.maxSpeed = 32});

    //score low goal
    chassis.turnToPoint(-11, -13, 500);
    chassis.moveToPoint(-11, -13, 500, {}, false);
    score_intake("low");
    pros::delay(1000);
    stop_all_intake_motors();
    activate_intake(true);

    //intake mid goal blocks
    chassis.moveToPoint(-22, -22, 500, {.forwards = false});
    chassis.turnToPoint(-22, 10, 500);
    chassis.moveToPoint(-22, 10, 750, {.minSpeed = 32, .earlyExitRange = 1});
    chassis.moveToPoint(-22, 22, 1500, {.maxSpeed = 32});

}

void left_awp_r4_auton() {
    chassis.setPose(-61.625, 16.185, 90);

    //move to left wall loader
    chassis.moveToPoint(-47, 16.185, 500);
    chassis.turnToHeading(0, 500);
    chassis.moveToPoint(-47, 49, 500, {}, false);
    activate_wall_loading();
    chassis.turnToHeading(270, 500, {}, false);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);

    //intake from wall loader
    activate_intake(true);
    chassis.moveToPoint(-60, 49, 500, {.maxSpeed = 64}, false);
    pros::delay(150);
    chassis.moveToPoint(-61, 49, 100, {}, false);
    pros::delay(150);
    chassis.moveToPoint(-61, 49, 100);

    //score left high goal
    chassis.moveToPoint(-28, 49, 750, {.forwards = false}, false);
    deactivate_wall_loading();
    score_intake("high");
    pros::delay(1000);
    stop_all_intake_motors();
    chassis.moveToPoint(-29, 49, 100, {}, false);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);

    //reset position and intake mid goal blocks
    chassis.setPose(-28, 47, 270);
    chassis.moveToPoint(-35, 47, 500);
    chassis.turnToPoint(-28, 33, 500, {}, false);
    activate_intake(true);
    chassis.moveToPoint(-28, 33, 750, {.minSpeed = 32, .earlyExitRange = 1});
    chassis.moveToPoint(-22, 22, 1500, {.maxSpeed = 32});

    //score mid goal
    chassis.turnToPoint(-13, 13, 500, {.forwards = false});
    chassis.moveToPoint(-13, 13, 500, {}, false);
    score_intake("mid");
    pros::delay(1000);
    stop_all_intake_motors();

    //intake low goal blocks
    chassis.moveToPoint(-22, 22, 500);
    chassis.turnToPoint(-22, -10, 500, {}, false);
    activate_intake(true);
    chassis.moveToPoint(-22, -10, 750, {.minSpeed = 32, .earlyExitRange = 1});
    chassis.moveToPoint(-22, -22, 1500, {.maxSpeed = 32});

    //move to right wall loader
    chassis.turnToPoint(-47, -50, 500);
    chassis.moveToPoint(-47, -50, 750, {}, false);
    activate_wall_loading();
    chassis.turnToHeading(270, 250, {}, false);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);

    //intake from wall loader
    chassis.moveToPoint(-64, -50, 500, {.maxSpeed = 64}, false);
    pros::delay(150);
    chassis.moveToPoint(-65, -50, 100, {}, false);
    pros::delay(150);
    chassis.moveToPoint(-65, -50, 100);

    //score right high goal
    chassis.moveToPoint(-26, -50, 750, {.forwards = false}, false);
    score_intake("high");
}

void right_awp_auton() {
    chassis.setPose(-61.625, -16.185, 90);
    activate_intake(true);

    //move to mid goal balls and intake
    chassis.moveToPoint(-45, -16.185, 500);
    chassis.turnToPoint(-28, 12, 500);
    chassis.moveToPoint(-28, 12, 750, {.minSpeed = 32, .earlyExitRange = 1});
    chassis.moveToPoint(-21, 24, 1750, {.maxSpeed = 32}, false);

    //move ot mid goal and outtake
    chassis.turnToPoint(-9.5, 15, 500, {.forwards = false});
    chassis.moveToPoint(-9, 14.5, 500, {.forwards = false}, false);
    score_intake("mid");
    pros::delay(750);
    stop_scoring();

    //move to low goal blocks and intake
    chassis.moveToPoint(-16, 16, 500);
    chassis.turnToPoint(-19, -11, 500);
    activate_intake(true);
    chassis.moveToPoint(-19, -11, 750, {.minSpeed = 32, .earlyExitRange = 1});
    chassis.moveToPoint(-19, -24, 1000, {.maxSpeed = 32}, false);
    stop_intake();

    //move to low goal and outtake
    chassis.turnToPoint(-9.5, -14, 500, {}, false);
    chassis.moveToPoint(-9.5, -14, 500, {}, false);
    score_intake("low");
    pros::delay(750);
    stop_all_intake_motors();
    
    //move to wall loader
    chassis.turnToPoint(-45, -46, 500, {.forwards = false}, false);
    activate_intake(true);
    chassis.moveToPoint(-45, -46, 1500, {.forwards = false}, false);
    activate_wall_loading();
    chassis.turnToHeading(270, 500, {}, false);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);

    //intake balls from loader
    chassis.moveToPoint(-64, -46, 750, {.maxSpeed = 64});
    pros::delay(250);
    chassis.moveToPoint(-65, -46, 100, {}, false);
    pros::delay(250);
    chassis.moveToPoint(-65, -46, 100, {}, false);
  
    //score balls in high goal
    chassis.turnToPoint(-28, -47, 250, {.forwards = false});
    chassis.moveToPoint(-28, -47, 750, {.forwards = false}, false);
    deactivate_wall_loading();
    score_intake("high");
}


void left_awp_auton() {
    chassis.setPose(-61.625, 16.185, 90);
    activate_intake(true);

    //move to low goal balls and intake
    chassis.moveToPoint(-45, 16.185, 500);
    chassis.turnToPoint(-28, -12, 500);
    chassis.moveToPoint(-28, -12, 750, {.minSpeed = 32, .earlyExitRange = 1});
    chassis.moveToPoint(-21, -24, 1750, {.maxSpeed = 32});

    //move ot low goal and outtake
    chassis.turnToPoint(-9.5, -15, 500);
    chassis.moveToPoint(-9.5, -15, 500, {}, false);
    score_intake("low");
    pros::delay(750);
    stop_scoring();

    //move to mid goal blocks and intake
    chassis.moveToPoint(-16, -16, 500, {.forwards = false});
    chassis.turnToPoint(-19, 11, 500);
    chassis.moveToPoint(-19, 11, 750, {.minSpeed = 32, .earlyExitRange = 1});
    chassis.moveToPoint(-19, 24, 1750, {.maxSpeed = 32});

    //move to mid goal and outtake
    chassis.turnToPoint(-10, 14, 500, {.forwards = false});
    chassis.moveToPoint(-10, 15, 500, {.forwards = false}, false);
    score_intake("mid");
    pros::delay(750);
    stop_all_intake_motors();
    
    //move to wall loader
    chassis.moveToPoint(-47, 46, 1000);
    chassis.turnToHeading(270, 500, {}, false);
    activate_wall_loading();

    //intake balls from loader
    activate_intake(true);
    chassis.moveToPoint(-60, 47, 250, {}, false);
    chassis.moveToPoint(-56, 47, 150, {.forwards = false, .maxSpeed = 64});
    chassis.moveToPoint(-60, 47, 150);
    chassis.moveToPoint(-56, 47, 150, {.forwards = false, .maxSpeed = 64});
    chassis.moveToPoint(-60, 47, 150);
  
    //score balls in high goal
    chassis.moveToPoint(-28, 47, 750, {.forwards = false}, false);
    score_intake("high");
}

void right_auton() {
    chassis.setPose(-61.625, -16.185, 90);
    activate_intake(true);

    //move to balls and intake
    chassis.moveToPoint(-34, -16.185, 1000, {}, false);
    chassis.turnToPoint(-18, -25, 500);
    chassis.moveToPoint(-18, -25, 2000, {.maxSpeed = 36});

    //go to low goal
    chassis.turnToPoint(-9, -14.5, 750);
    chassis.moveToPoint(-9, -14.5, 750, {}, false);
    score_intake("low");
    pros::delay(1000);
    stop_all_intake_motors();
    activate_intake(true);

    //move to wall loader
    chassis.turnToPoint(-45, -49, 500, {.forwards = false});
    chassis.moveToPoint(-45, -49, 1500, {.forwards = false}, false);
    activate_wall_loading();
    chassis.turnToHeading(270, 750);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);

    //intake from wall loader
    chassis.moveToPoint(-64, -49, 750, {.maxSpeed = 64}, false);
    pros::delay(250);
    chassis.moveToPoint(-65, -49, 100, {}, false);
    pros::delay(250);
    chassis.moveToPoint(-65, -49, 100, {}, false);
    pros::delay(250);
    chassis.moveToPoint(-65, -49, 100, {}, false);

    /*chassis.moveToPoint(-53, -49, 200, {.forwards = false, .maxSpeed = 64});
    chassis.moveToPoint(-58, -49, 200);
    chassis.moveToPoint(-53, -49, 200, {.forwards = false, .maxSpeed = 64});
    chassis.moveToPoint(-58, -49, 200);
    chassis.moveToPoint(-53, -49, 200, {.forwards = false, .maxSpeed = 64});
    chassis.moveToPoint(-58, -49, 200);
    chassis.moveToPoint(-53, -49, 200, {.forwards = false, .maxSpeed = 64});
    chassis.moveToPoint(-58, -49, 200);*/
    
    
    /*chassis.turnToHeading(275, 100);
    chassis.turnToHeading(265, 100);
    chassis.turnToHeading(275, 100);
    chassis.turnToHeading(265, 100);
    chassis.turnToHeading(275, 100);
    chassis.turnToHeading(270, 100);*/
    //pros::delay(600);

    //move to high goal and score
    chassis.moveToPoint(-23, -49, 1500, {.forwards = false}, false);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    deactivate_wall_loading();
    score_intake("high");
    pros::delay(1500);
    chassis.moveToPoint(-22, -49, 100, {.forwards = false});
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);

    //reset position and move to descore
    chassis.setPose(-28, -46, 270);
    stop_all_intake_motors();
    chassis.moveToPoint(-42, -33, 1000);
    chassis.turnToHeading(90, 500, {});
    chassis.moveToPoint(-6, -33, 3000);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);

}

void right_long_auton() {
    chassis.setPose(-61.625, -16.185, 90);
    activate_intake(true);

    //move to balls and intake
    chassis.moveToPoint(-34, -16.185, 1000, {}, false);
    chassis.turnToPoint(-18, -25, 500);
    chassis.moveToPoint(-18, -25, 3000, {.maxSpeed = 24});


    //move to wall loader
    chassis.turnToPoint(-45, -49, 500, {}, false);
    stop_intake();
    chassis.moveToPoint(-45, -49, 1500, {}, false);
    activate_wall_loading();
    chassis.turnToHeading(270, 500);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    activate_intake(true);

    //intake from wall loader
    chassis.moveToPoint(-64, -49, 750, {.maxSpeed = 64}, false);
    pros::delay(250);
    chassis.moveToPoint(-65, -49, 100, {}, false);
    pros::delay(250);
    chassis.moveToPoint(-65, -49, 100, {}, false);
    pros::delay(250);
    chassis.moveToPoint(-65, -49, 100, {}, false);

    /*chassis.moveToPoint(-52, -49, 200, {.forwards = false, .maxSpeed = 64});
    chassis.moveToPoint(-57, -49, 200);
    chassis.moveToPoint(-52, -49, 200, {.forwards = false, .maxSpeed = 64});
    chassis.moveToPoint(-57, -49, 200);
    chassis.moveToPoint(-52, -49, 200, {.forwards = false, .maxSpeed = 64});
    chassis.moveToPoint(-57, -49, 200);
    chassis.moveToPoint(-52, -49, 200, {.forwards = false, .maxSpeed = 64});
    chassis.moveToPoint(-57, -49, 200);*/
    
    
    /*chassis.turnToHeading(275, 100);
    chassis.turnToHeading(265, 100);
    chassis.turnToHeading(275, 100);
    chassis.turnToHeading(265, 100);
    chassis.turnToHeading(275, 100);
    chassis.turnToHeading(270, 100);*/
    //pros::delay(600);

    //move to high goal and score
    chassis.moveToPoint(-23, -49, 1500, {.forwards = false}, false);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    deactivate_wall_loading();
    score_intake("high");
    pros::delay(3500);
    stop_all_intake_motors();
    chassis.moveToPoint(-22, -49, 100, {.forwards = false});
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);

    //reset position and move to descore
    chassis.setPose(-28, -46, 270);
    chassis.moveToPoint(-42, -33, 1000);
    chassis.turnToHeading(90, 500, {});
    activate_right_descore();
    chassis.moveToPoint(-6, -33, 3000);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
}

void left_auton() {
    chassis.setPose(-61.625, 16.185, 90);
    activate_intake(true);

    //move to balls and intake
    chassis.moveToPoint(-34, 16.185, 1000, {}, false);
    chassis.turnToPoint(-18, 25, 500);
    chassis.moveToPoint(-18, 25, 2000, {.maxSpeed = 36});

    //go to mid goal
    chassis.turnToPoint(-7, 12, 750, {.forwards = false});
    chassis.moveToPoint(-7, 12, 750, {.forwards = false}, false);
    score_intake("mid");
    pros::delay(1000);
    stop_all_intake_motors();

    //move to wall loader
    chassis.turnToPoint(-45, 50, 500);
    chassis.moveToPoint(-45, 50, 1500, {}, false);
    activate_wall_loading();
    chassis.turnToHeading(270, 750);
    activate_intake(true);
    
    chassis.moveToPoint(-56, 50, 500, {.maxSpeed = 64}, false);

    chassis.moveToPoint(-52, 50, 200, {.forwards = false, .maxSpeed = 64});
    chassis.moveToPoint(-56, 50, 200);
    chassis.moveToPoint(-52, 50, 200, {.forwards = false, .maxSpeed = 64});
    chassis.moveToPoint(-56, 50, 200);
    chassis.moveToPoint(-52, 50, 200, {.forwards = false, .maxSpeed = 64});
    chassis.moveToPoint(-56, 50, 200);
    chassis.moveToPoint(-52, 50, 200, {.forwards = false, .maxSpeed = 64});
    chassis.moveToPoint(-56, 50, 200);
    
    
    /*chassis.turnToHeading(275, 100);
    chassis.turnToHeading(265, 100);
    chassis.turnToHeading(275, 100);
    chassis.turnToHeading(265, 100);
    chassis.turnToHeading(275, 100);
    chassis.turnToHeading(270, 100);*/
    //pros::delay(600);

    //move to high goal and score
    chassis.moveToPoint(-22, 50, 1500, {.forwards = false}, false);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    deactivate_wall_loading();
    score_intake("high");
    pros::delay(1000);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);

    //reset position and move to descore
    chassis.setPose(-28, 46, 270);
    stop_all_intake_motors();
    chassis.moveToPoint(-42, 59, 1000);
    chassis.turnToHeading(90, 500, {});
    activate_right_descore();
    chassis.moveToPoint(-8, 59, 3000);
    pros::delay(650);
    deactivate_right_descore();
}

void left_long_auton() {
    chassis.setPose(-61.625, 16.185, 90);
    activate_intake(true);

    //move to balls and intake
    chassis.moveToPoint(-34, 16.185, 1000, {}, false);
    chassis.turnToPoint(-18, 25, 500);
    chassis.moveToPoint(-18, 25, 2000, {.maxSpeed = 30});

    //move to wall loader and intake
    chassis.turnToPoint(-45, 50, 500);
    chassis.moveToPoint(-45, 50, 1500, {}, false);
    activate_wall_loading();
    chassis.turnToHeading(270, 500);
    activate_intake(true);
    chassis.moveToPoint(-56, 50, 500, {.maxSpeed = 64}, false);

    chassis.moveToPoint(-52, 50, 200, {.forwards = false, .maxSpeed = 64});
    chassis.moveToPoint(-56, 50, 200);
    chassis.moveToPoint(-52, 50, 200, {.forwards = false, .maxSpeed = 64});
    chassis.moveToPoint(-56, 50, 200);
    chassis.moveToPoint(-52, 50, 200, {.forwards = false, .maxSpeed = 64});
    chassis.moveToPoint(-56, 50, 200);
    chassis.moveToPoint(-52, 50, 200, {.forwards = false, .maxSpeed = 64});
    chassis.moveToPoint(-56, 50, 200);
    
    
    /*chassis.turnToHeading(275, 100);
    chassis.turnToHeading(265, 100);
    chassis.turnToHeading(275, 100);
    chassis.turnToHeading(265, 100);
    chassis.turnToHeading(275, 100);
    chassis.turnToHeading(270, 100);*/
    //pros::delay(600);

    //move to high goal and score
    chassis.moveToPoint(-23, 50, 1500, {.forwards = false}, false);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    deactivate_wall_loading();
    score_intake("high");
    pros::delay(3500);
    stop_all_intake_motors();
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);

    //reset position and move to descore
    chassis.setPose(-28, 46, 270);
    chassis.moveToPoint(-42, 59, 1000);
    chassis.turnToHeading(90, 500, {});
    activate_right_descore();
    chassis.moveToPoint(-8, 59, 3000);
    pros::delay(700);
    deactivate_right_descore();
}   

void red_left() {
    alliance_color = "red";
    console.print("This is red left");
    left_auton();
}

void red_left_long() {
    alliance_color = "red";
    console.print("This is red left long");
    left_long_auton();
}

void red_left_awp() {
    alliance_color = "red";
    console.print("This is red left AWP");
    left_awp_auton();
}

void red_left_awp_r4() {
    alliance_color = "red";
    console.print("This is red left R4 AWP");
    left_awp_r4_auton();
}

void red_right() {
    alliance_color = "red";
    console.print("This is red right");
    right_auton();
}

void red_right_long() {
    alliance_color = "red";
    console.print("This is red right long");
    right_long_auton();
}

void red_right_awp() {
    alliance_color = "red";
    console.print("This is red right AWP");
    right_awp_auton();
}

void red_right_awp_r4() {
    alliance_color = "red";
    console.print("This is red right R4 AWP");
    right_awp_r4_auton();
}

void blue_left() {
    alliance_color = "blue";
    console.print("This is blue left");
    left_auton();
}

void blue_left_long() {
    alliance_color = "blue";
    console.print("This is blue left long");
    left_long_auton();
}

void blue_left_awp() {
    alliance_color = "blue";
    console.print("This is blue left AWP");
    left_awp_auton();
}

void blue_left_awp_r4() {
    alliance_color = "blue";
    console.print("This is blue left R4 AWP");
    left_awp_r4_auton();
}

void blue_right() {
    alliance_color = "blue";
    console.print("This is blue right");
    right_auton();
}

void blue_right_long() {
    alliance_color = "blue";
    console.print("This is blue right long");
    right_long_auton();
}

void blue_right_awp() {
    alliance_color = "blue";
    console.print("This is blue right AWP");
    right_awp_auton();
}

void blue_right_awp_r4() {
    alliance_color = "blue";
    console.print("This is blue right R4 AWP");
    right_awp_r4_auton();
}

void auton_skills() {
    chassis.setPose(-61.625, -16.185, 90);
    
    //move to red right wall loader and intake
    chassis.moveToPoint(-47, -16.185, 750);
    chassis.turnToHeading(180, 500);
    chassis.moveToPoint(-47, -49, 1000, {.maxSpeed = 96});
    chassis.turnToHeading(270, 500);
    activate_wall_loading();
    activate_intake(true);
    chassis.moveToPoint(-58, -49, 500, {}, false);

    chassis.moveToPoint(-54, -49, 200, {.forwards = false});
    chassis.moveToPoint(-58, -49, 200);
    chassis.moveToPoint(-54, -49, 200, {.forwards = false});
    chassis.moveToPoint(-58, -49, 200);
    chassis.moveToPoint(-54, -49, 200, {.forwards = false});
    chassis.moveToPoint(-58, -49, 200);
    chassis.moveToPoint(-54, -49, 200, {.forwards = false});
    chassis.moveToPoint(-58, -49, 200);
    chassis.moveToPoint(-54, -49, 200, {.forwards = false});
    chassis.moveToPoint(-58, -49, 200);
    chassis.moveToPoint(-54, -49, 200, {.forwards = false});
    chassis.moveToPoint(-58, -49, 200);

    chassis.moveToPoint(-40, -49, 1000, {.forwards = false}, false);
    deactivate_wall_loading();

    chassis.turnToHeading(180, 500);
    chassis.moveToPoint(-40, -63, 500, {.maxSpeed = 96});
    chassis.turnToHeading(90, 500);
    chassis.moveToPoint(50, -63, 3000, {.maxSpeed = 96});
    chassis.turnToHeading(0, 500);
    chassis.moveToPoint(50, -50, 500, {.maxSpeed = 96});
    chassis.turnToHeading(90, 500);
    chassis.moveToPoint(36, -50, 1000, {.forwards = false, .maxSpeed = 96}, false);
    score_intake("high");
    activate_wall_loading();
    pros::delay(3000);
    stop_scoring();

    chassis.setPose(28,-46, 90);

    chassis.moveToPoint(57, -46, 750);

    chassis.moveToPoint(56, -46, 200, {.forwards = false});
    chassis.moveToPoint(60, -46, 200);
    chassis.moveToPoint(56, -46, 200, {.forwards = false});
    chassis.moveToPoint(60, -46, 200);
    chassis.moveToPoint(56, -46, 200, {.forwards = false});
    chassis.moveToPoint(60, -46, 200);
    chassis.moveToPoint(56, -46, 200, {.forwards = false});
    chassis.moveToPoint(60, -46, 200);
    chassis.moveToPoint(56, -46, 200, {.forwards = false});
    chassis.moveToPoint(60, -46, 200);
    chassis.moveToPoint(56, -46, 200, {.forwards = false});
    chassis.moveToPoint(60, -46, 200);

    chassis.moveToPoint(26, -46, 1000, {.forwards = false}, false);
    score_intake("high");
    deactivate_wall_loading();
    pros::delay(3000);
    stop_all_intake_motors();

    chassis.moveToPoint(34, -47, 750);
    chassis.turnToPoint(28.5, -34, 750, {}, false);
    activate_intake(true);
    chassis.moveToPoint(28.5, -34, 750, {.minSpeed = 32, .earlyExitRange = 1});
    chassis.moveToPoint(23.5, -24, 2000, {.maxSpeed = 32});

    /*
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
    */
}

void do_nothing_auton() {
    chassis.setPose(0,0,0);
    chassis.moveToPoint(0, 4, 2000);
}

void pid_auton() {
    chassis.setPose(0,0,0); 
    chassis.moveToPoint(0,48,100000);   
};