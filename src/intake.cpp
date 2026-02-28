#include "myapi/intake.hpp"
#include "main.h"
#include "myapi/pneumatics.hpp"

void activate_intake(bool direction) {
  if (direction) {
    activate_upper_scoring();
    intake_motor_1.move(127);
    intake_motor_2.move(-127);
    scoring_motor.move(-127);
    low_scoring = false;
    intaking = true;

  } else {
    intake_motor_1.move(-127);
    intake_motor_2.move(127);
    low_scoring = true;
    intaking = false;
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
  intaking = false;
  mid_scoring = true;
  high_scoring = false;
  low_scoring = false;
}

void score_intake(std::string goal) {
  if (goal == "low") {
    activate_upper_scoring();
    intake_motor_1.move(-127);
    pros::delay(100);
    intake_motor_2.move(127);
    scoring_motor.move(-127);
    low_scoring = true;
    mid_scoring = false;
    high_scoring = false;
    intaking = false;
  } else if (goal == "mid") {
    score_intake("low");
    pros::delay(200);
    activate_mid_scoring();
    deactivate_mid_descore();
    scoring_motor.move(-127);
    intake_motor_2.move(-127);
    pros::delay(100);
    intake_motor_1.move(127);
    mid_scoring = true;
    high_scoring = false;
    low_scoring = false;
    intaking = false;
  } else if (goal == "high") {
    activate_upper_scoring();
    scoring_motor.move(127);
    intake_motor_2.move(-127);
    pros::delay(100);
    intake_motor_1.move(127);
    intaking = false;
    high_scoring = true;
    mid_scoring = false;
    low_scoring = false;
  }
}

void mid_reject_intake() {
  /*
intake_motor_2.move(127);
activate_upper_scoring();
scoring_motor.move(127);
pros::delay(100);

intake_motor_2.move(-127);
// pros::delay(500);

std::string block_color = detect_color_for_mid();
while (alliance_color != block_color &&
       midgoal_color_sensor.get_proximity() > 200) {
  block_color = detect_color_for_mid();
  pros::delay(20);
}
pros::delay(200);

activate_mid_scoring();
scoring_motor.move(-127);*/
  console.println("I see a block!");

  activate_upper_scoring();
  scoring_motor.move(127);

  std::string block_color = detect_color_for_high();

  while (alliance_color != block_color &&
         highgoal_color_sensor.get_proximity() > 200) {
    block_color = detect_color_for_high();
    pros::delay(20);
  }
  console.println("========== STOPPING ============");
  console.println(std::to_string(highgoal_color_sensor.get_proximity()));
  console.println(std::to_string(alliance_color != block_color));
  /*console.println("Ball is going through the next phase... (400ms)");
  // scoring_motor.move(-127);
  int timeout = 0;
  while (midgoal_color_sensor.get_proximity() < 200 && timeout < 5) {
    timeout++;
    pros::delay(20);
  }
  console.println("Waiting for exit (400ms)");
  // scoring_motor.move(-127);
  timeout = 0;
  while (midgoal_color_sensor.get_proximity() > 200 && timeout < 20) {
    timeout++;
    pros::delay(20);
  }
  console.println(std::to_string(highgoal_color_sensor.get_proximity()));*/
  pros::delay(200);
  activate_mid_scoring();
  scoring_motor.move(-127);
  console.println("fin.");
}

void high_reject_intake() {
  scoring_motor.move(-127);

  // intake_motor_2.move(127);
  activate_mid_scoring();
  // pros::delay(150);

  // intake_motor_2.move(-127);
  // scoring_motor.move(-127);
  // pros::delay(200);
  std::string block_color = detect_color_for_high();
  console.println("I see a block!");
  while (alliance_color != block_color &&
         highgoal_color_sensor.get_proximity() > 200) {
    block_color = detect_color_for_high();
    pros::delay(20);
  }
  console.println("========== STOPPING ============");
  console.println(std::to_string(highgoal_color_sensor.get_proximity()));
  console.println(std::to_string(alliance_color != block_color));
  console.println("Waiting to stop rejection (400ms)");
  // scoring_motor.move(-127);
  int timeout = 0;
  while (midgoal_color_sensor.get_proximity() > 200 && timeout < 20) {
    timeout++;
    pros::delay(20);
  }
  pros::delay(50);
  console.println("Activating scoring (150)");
  activate_upper_scoring();
  pros::delay(150);

  console.println("Moving scoring motor (150)");
  scoring_motor.move(127);
}

std::string detect_color_for_mid() {
  int rgb_value = midgoal_color_sensor.get_hue();
  if (rgb_value >= 190 && rgb_value <= 240) {
    return "blue";
  } else if (rgb_value <= 40) {
    return "red";
  } else {
    return "";
  }
}

std::string detect_color_for_high() {
  int rgb_value = highgoal_color_sensor.get_hue();
  if (rgb_value >= 190 && rgb_value <= 240) {
    return "blue";
  } else if (rgb_value <= 40) {
    return "red";
  } else {
    return "";
  }
}

void mid_auto_reject(void *) {
  while (true) {
    if (enable_auto_reject && mid_scoring) {
      std::string block_color = detect_color_for_high();
      if (alliance_color == "red" && block_color == "blue" &&
          highgoal_color_sensor.get_proximity() > 200) {
        mid_reject_intake();
      } else if (alliance_color == "blue" && block_color == "red" &&
                 highgoal_color_sensor.get_proximity() > 200) {
        mid_reject_intake();
      }
    }
    pros::delay(20);
  }
}

void high_auto_reject(void *) {
  int timeout = 0;
  while (true) {
    if (enable_auto_reject && high_scoring) {
      std::string block_color = detect_color_for_high();
      if (alliance_color == "red" && block_color == "blue" &&
          highgoal_color_sensor.get_proximity() > 200) {
        high_reject_intake();
      } else if (alliance_color == "blue" && block_color == "red" &&
                 highgoal_color_sensor.get_proximity() > 200) {
        high_reject_intake();
      }
    }
    pros::delay(20);
  }
}

/*void stop_intake_stalling() {
    while (true) {
        if(intaking) {
            if(intake_motor_1.get_current_draw() > 2.0 &&
intake_motor_1.get_actual_velocity() < 10) { int target_velocity =
intake_motor_1.get_target_velocity(); intake_motor_1.move(-1 * target_velocity);
                pros::delay(50);
                intake_motor_1.move(target_velocity);
            }

            if(intake_motor_2.get_current_draw() > 2.0 &&
intake_motor_2.get_actual_velocity() < 10) { int target_velocity =
intake_motor_2.get_target_velocity(); intake_motor_2.move(-1 * target_velocity);
                pros::delay(50);
                intake_motor_2.move(target_velocity);
            }

            //if(intake_motor_3.get_current_draw() > 2.0 &&
intake_motor_3.get_actual_velocity() < 10) {
              //  int target_velocity = intake_motor_3.get_target_velocity();
                //intake_motor_3.move(-1 * target_velocity);
                //pros::delay(50);
                //intake_motor_3.move(target_velocity);
            //}
        }

        if(scoring) {
            if(scoring_motor.get_current_draw() > 2.0 &&
scoring_motor.get_actual_velocity() < 5) { int target_velocity =
scoring_motor.get_target_velocity(); scoring_motor.move(-1 * target_velocity);
                pros::delay(50);
                scoring_motor.move(target_velocity);
            }
        }
    }
}*/

void toggle_auto_reject() { 
  enable_auto_reject = !enable_auto_reject; 
  if (enable_auto_reject) {
    controller.print(0, 0, "RejON");
  } else {
    controller.print(0, 0, "RejOFF");
  }
}

void overheating_protector() {
  int timeout;
  while (true) {
    while (intaking) {
      if (highgoal_color_sensor.get_proximity() > 200) {
        timeout++;
        if (timeout > 20) {
          intake_motor_2.brake();
          pros::delay(200);
        }
      } else {
        if (low_scoring) {
          intake_motor_2.move(127);
        } else {
          intake_motor_2.move(-127);
        }
        timeout = 0;
      }
      pros::delay(20);
    }
    pros::delay(20);
  }
}