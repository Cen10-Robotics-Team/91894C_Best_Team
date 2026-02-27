#include "main.h"

void activate_intake(bool direction){
    if(direction) {
        activate_upper_scoring();
        intake_motor_1.move(127);
        intake_motor_2.move(-127);
        scoring_motor.move(-127);
        intaking = true;

    } else {
        intake_motor_1.move(-127);  
        intake_motor_2.move(127);
        
    }
}

void stop_intake() {
    intake_motor_1.brake();
    intake_motor_2.brake();


    intaking = false;
    low_scoring = false;
}

void stop_scoring() {
    scoring_motor.brake();
    activate_upper_scoring();

    mid_scoring = false;
    high_scoring = false;
}

void stop_all_intake_motors() {
    stop_intake();
    stop_scoring();
}

void slow_mid_score() {
    score_intake("low");
    pros::delay(200);
    activate_mid_scoring();
    scoring_motor.move(-64);
    intake_motor_2.move(-127);
    pros::delay(50);
    intake_motor_1.move(127);
    intaking = true;
    mid_scoring = true;
    high_scoring = false;
    low_scoring = false;
}

void score_intake(std::string goal) {
    if(goal == "low") {
        activate_upper_scoring();
        intake_motor_1.move(-127);
        pros::delay(100);
        intake_motor_2.move(127);
        scoring_motor.move(-127);
        low_scoring = true;
        mid_scoring = false;
        high_scoring = false;
    } else if (goal == "mid") {
        score_intake("low");
        pros::delay(200);

        activate_mid_scoring();
        scoring_motor.move(-127);
        intake_motor_2.move(-127);
        pros::delay(50);
        intake_motor_1.move(127);
        intaking = true;
        mid_scoring = true;
        high_scoring = false;
        low_scoring = false;
    } else if (goal == "high") {
        activate_upper_scoring();
        scoring_motor.move(127);
        intake_motor_2.move(-127);
        pros::delay(100);
        intake_motor_1.move(127);
        intaking = true;
        high_scoring = true;
        mid_scoring = false;
        low_scoring = false;
    }
}

void mid_reject_intake() {
    intake_motor_2.move(127);
    activate_upper_scoring();
    scoring_motor.move(127);
    pros::delay(100);
    
    intake_motor_2.move(-127);
    //pros::delay(500);

    std::string block_color = detect_color();
    while (alliance_color != block_color && color_sensor.get_proximity() > 200) {
        block_color = detect_color();
        pros::delay(20);
    }
    pros::delay(200);
    
    activate_mid_scoring();
    scoring_motor.move(-127);
}

void high_reject_intake() {
    scoring_motor.move(-127);

    //intake_motor_2.move(127);
    activate_mid_scoring();
    //pros::delay(150);

    //intake_motor_2.move(-127);
    //scoring_motor.move(-127);
    //pros::delay(200);
    std::string block_color = detect_color();
    while (alliance_color != block_color && color_sensor.get_proximity() > 200) {
        block_color = detect_color();
        pros::delay(20);
    }
    pros::delay(200);

    activate_upper_scoring();
    pros::delay(150);

    scoring_motor.move(127);

}

std::string detect_color() {
    int rgb_value = color_sensor.get_hue();
    if(rgb_value >= 190 && rgb_value <= 240) {
        return "blue";
    } else if (rgb_value <= 40) {
        return "red";
    } else {
        return "";
    }
}

void mid_auto_reject(void*) {
    while(true){
        if(enable_auto_reject && mid_scoring) {
            std::string block_color = detect_color();
            if(alliance_color == "red" && block_color == "blue" && color_sensor.get_proximity() > 200) {
                mid_reject_intake();
            } else if (alliance_color == "blue" && block_color == "red" && color_sensor.get_proximity() > 200) {
                mid_reject_intake();
            }
        }
        pros::delay(20);
    }
}

void high_auto_reject(void*) {
    while(true) {
        if(enable_auto_reject && high_scoring) {
            std::string block_color = detect_color();
            if(alliance_color == "red" && block_color == "blue" && color_sensor.get_proximity() > 200) {
                high_reject_intake();
            } else if (alliance_color == "blue" && block_color == "red" && color_sensor.get_proximity() > 200) {
                high_reject_intake();
            }
        }
        pros::delay(20);
    }
}

/*void stop_intake_stalling() {
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
}*/

void toggle_auto_reject() {
    enable_auto_reject = !enable_auto_reject;
}