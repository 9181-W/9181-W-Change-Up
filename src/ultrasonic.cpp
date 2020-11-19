#include "okapi/api.hpp"
using namespace okapi;

#define ULTRASONIC_PORT_INPUT 'A'
#define ULTRASONIC_PORT_OUTPUT 'B'


pros::ADIUltrasonic* ultrasonic_1 = NULL;

double ultrasonic_value = 0.0;

void ultrasonic_reading(void* param)
{
  while (true)
  {
    ultrasonic_value = ultrasonic_1->get_value();
    //pros::lcd::print(2,"Ultrasonic Value %f",ultrasonic_value);
    pros::delay(33);
  }
}

void ultrasonic_initialize()
{
  ultrasonic_1 = new pros::ADIUltrasonic(ULTRASONIC_PORT_INPUT, ULTRASONIC_PORT_OUTPUT);

  pros::Task ultrasonic_task (ultrasonic_reading, (void*)"PROSV5", TASK_PRIORITY_DEFAULT,
    TASK_STACK_DEPTH_DEFAULT, "Ultrasonic Task");
}

double get_ultrasonic_value()
{
  return ultrasonic_value;
}
