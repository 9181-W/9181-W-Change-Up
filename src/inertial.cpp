#include "okapi/api.hpp"
using namespace okapi;

#define INERTIAL_PORT 15

pros::Imu* inertial_1 = NULL;

double inertial_value = 0.0;

void inertial_reading(void* param)
{
  while (true)
  {
    if (inertial_1->is_calibrating() == true)
    {
      inertial_value = 0;
    }
    else
    {
      inertial_value = inertial_1->get_rotation();
    }

    pros::lcd::print(7,"Inertial Value %f",inertial_value);
    pros::delay(33);
  }
}

void inertial_initialize()
{
  inertial_1 = new pros::Imu(INERTIAL_PORT);

  while (true)
  {
    if (inertial_1->is_calibrating() == false)
    {
      break;
    }

    pros::delay(33);

  }

  pros::Task inertial_value_task (inertial_reading, (void*)"PROSV5", TASK_PRIORITY_DEFAULT,
    TASK_STACK_DEPTH_DEFAULT, "Inertial Value Task");
}

void inertial_reset()
{
  inertial_1->reset();
}

double inertial_get_value()
{
  return inertial_value;
}
