#include "opcontrol.hpp"
#include "ultrasonic.hpp"
#include "okapi/api.hpp"
#include "initialize.hpp"

void tank_drive(double gearset_rpm = 200)
{
  double left_speed = master_controller.getAnalog(okapi::ControllerAnalog::leftY) * gearset_rpm;
  double right_speed = master_controller.getAnalog(okapi::ControllerAnalog::rightY) * gearset_rpm;

  left_rear_mtr.moveVelocity(left_speed);
  left_front_mtr.moveVelocity(left_speed);
  right_rear_mtr.moveVelocity(-right_speed);
  right_front_mtr.moveVelocity(-right_speed);
}

void arcade_drive(double gearset_rpm = 200)
{
  // getAnalog returns a float -1,1
  double leftY = master_controller.getAnalog(okapi::ControllerAnalog::leftY);
  double rightX = master_controller.getAnalog(okapi::ControllerAnalog::rightX);
  double leftX = master_controller.getAnalog(okapi::ControllerAnalog::leftX);

  //calculate individual wheel speeds between -1,1
  /*
  double front_left_speed = rightX + leftY + leftX;
  double front_right_speed = rightX - leftY + leftX;
  double back_left_speed = rightX + leftY - leftX;
  double back_right_speed = rightX - leftY - leftX;
  */

  double front_left_speed = leftY + rightX + leftX;
  double front_right_speed = leftY - rightX - leftX;
  double back_left_speed = leftY + rightX - leftX;
  double back_right_speed = leftY - rightX + leftX;

  //calculate wheel speeds in rpm
  front_left_speed *= gearset_rpm;
  front_right_speed *= gearset_rpm;
  back_left_speed *= gearset_rpm;
  back_right_speed *= gearset_rpm;

  front_left_speed = std::clamp(front_left_speed, -gearset_rpm, gearset_rpm);
  front_right_speed = std::clamp(front_right_speed, -gearset_rpm, gearset_rpm);
  back_left_speed = std::clamp(back_left_speed, -gearset_rpm, gearset_rpm);
  back_right_speed = std::clamp(back_right_speed, -gearset_rpm, gearset_rpm);

  left_front_mtr.moveVelocity(front_left_speed);
  right_front_mtr.moveVelocity(front_right_speed);
  left_rear_mtr.moveVelocity(back_left_speed);
  right_rear_mtr.moveVelocity(back_right_speed);
}

void intake_controls()
{
  if ((master_controller.getDigital(okapi::ControllerDigital::R1)) == true)
  {
    left_intake_mtr.moveVelocity(200);
    right_intake_mtr.moveVelocity(-200);
  }

  else if ((master_controller.getDigital(okapi::ControllerDigital::L1)) == true)
  {
    left_intake_mtr.moveVelocity(-200);
    right_intake_mtr.moveVelocity(200);
  }

  else if ((master_controller.getDigital(okapi::ControllerDigital::L1)) && (master_controller.getDigital(okapi::ControllerDigital::R2)) == true)
  {
    left_intake_mtr.moveVelocity(-200);
    right_intake_mtr.moveVelocity(200);
  }

  else
  {
    left_intake_mtr.moveVelocity(0);
    right_intake_mtr.moveVelocity(0);
  }
}

void bottom_controls()
{
  if ((master_controller.getDigital(okapi::ControllerDigital::R2)) == true)
  {
    bottom_mtr.moveVelocity(-300);
  }

  else if (((master_controller.getDigital(okapi::ControllerDigital::R1)) == true) && (get_ultrasonic_value() < 75))
  {
    bottom_mtr.moveVelocity(0);
  }

  else if ((master_controller.getDigital(okapi::ControllerDigital::R1)) == true)
  {
    bottom_mtr.moveVelocity(-300);
  }

  else if ((master_controller.getDigital(okapi::ControllerDigital::L1)) == true)
  {
    bottom_mtr.moveVelocity(300);
  }

  else if ((master_controller.getDigital(okapi::ControllerDigital::R1)) && (master_controller.getDigital(okapi::ControllerDigital::R2)) == true)
  {
    bottom_mtr.moveVelocity(-300);
  }

  else
  {
    bottom_mtr.moveVelocity(0);
  }
}

void top_controls()
{
  if ((master_controller.getDigital(okapi::ControllerDigital::L2)) == true)
  {
    top_mtr.moveVelocity(-400);
  }

  else if ((master_controller.getDigital(okapi::ControllerDigital::R2)) == true)
  {
    top_mtr.moveVelocity(600);
  }

  else if (((master_controller.getDigital(okapi::ControllerDigital::R1)) == true) && (get_ultrasonic_value() < 75))
  {
    top_mtr.moveVelocity(0);
  }


  else if ((master_controller.getDigital(okapi::ControllerDigital::L1)) == true)
  {
    top_mtr.moveVelocity(-600);
  }

  else if ((master_controller.getDigital(okapi::ControllerDigital::L2)) && (master_controller.getDigital(okapi::ControllerDigital::R2)) == true)
  {
    top_mtr.moveVelocity(600);
  }

  else if ((master_controller.getDigital(okapi::ControllerDigital::R1)) == true)
  {
    top_mtr.moveVelocity(150);
  }

  else
  {
    top_mtr.moveVelocity(0);
  }

}

void modified_opcontrol()
{
  while (true)
  {
    arcade_drive();
    intake_controls();
    bottom_controls();
    top_controls();

    pros::delay(33);
  }
}
