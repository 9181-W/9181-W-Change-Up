#include "autonomous.hpp"
#include "initialize.hpp"
#include "odom.hpp"

void modified_autonomous()
{
	/*
	right_rear_mtr.moveVelocity(200);
	left_rear_mtr.moveVelocity(200);
	right_front_mtr.moveVelocity(200);
	left_front_mtr.moveVelocity(200);

	pros::delay(1000);

	right_rear_mtr.moveVelocity(0);
	left_rear_mtr.moveVelocity(0);
	right_front_mtr.moveVelocity(0);
	left_front_mtr.moveVelocity(0);
	*/

	drive_to_point(0_ft, 1_ft);
}
