#include "main.h"

void activate_intake(bool direction){
    if(direction) {
        activate_upper_scoring();
        intake_motor_1.move(127);
        intake_motor_2.move(-127);
        scoring_motor.move(-127);
        

    } else {
        intake_motor_1.move(-127);  
        intake_motor_2.move(127);
        
    }
    intaking = true;
}

void stop_intake() {
    intake_motor_1.brake();
    intake_motor_2.brake();


    intaking = false;
}

void stop_scoring() {
    scoring_motor.brake();
    activate_upper_scoring();

    scoring = false;
}

void stop_all_intake_motors() {
    stop_intake();
    stop_scoring();
}

void score_intake(std::string goal) {
    if(goal == "low") {
        activate_upper_scoring();
        intake_motor_1.move(-127);
        pros::delay(100);
        intake_motor_2.move(127);
        scoring_motor.move(-127);
        scoring = true;
    } else if (goal == "mid") {
        score_intake("low");
        pros::delay(200);

        activate_mid_scoring();
        scoring_motor.move(-127);
        intake_motor_2.move(-127);
        pros::delay(100);
        intake_motor_1.move(127);
        scoring = true;
        intaking = true;
    } else if (goal == "high") {
        activate_upper_scoring();
        scoring_motor.move(127);
        intake_motor_2.move(-127);
        pros::delay(100);
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

            //if(intake_motor_3.get_current_draw() > 2.0 && intake_motor_3.get_actual_velocity() < 10) {
              //  int target_velocity = intake_motor_3.get_target_velocity();
                //intake_motor_3.move(-1 * target_velocity);
                //pros::delay(50);
                //intake_motor_3.move(target_velocity);
            //}
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