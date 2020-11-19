#include "initialize.hpp"
#include "okapi/api.hpp"
#include "position_tracker.hpp"
#include "inertial.hpp"
#include "ultrasonic.hpp"
#include "firmware.hpp"
#include "odom.hpp"
using namespace okapi;

#define RIGHT_REAR_WHEEL_PORT 20
#define LEFT_REAR_WHEEL_PORT 11
#define RIGHT_FRONT_WHEEL_PORT 10
#define LEFT_FRONT_WHEEL_PORT 1
#define LEFT_INTAKE_PORT 17
#define RIGHT_INTAKE_PORT 18
#define BOTTOM_PORT 19
#define TOP_PORT 21

Controller master_controller(ControllerId::master);

Motor right_rear_mtr(RIGHT_REAR_WHEEL_PORT, true, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);
Motor left_rear_mtr(LEFT_REAR_WHEEL_PORT, false, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);
Motor right_front_mtr(RIGHT_FRONT_WHEEL_PORT, true, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);
Motor left_front_mtr(LEFT_FRONT_WHEEL_PORT, false, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);
Motor left_intake_mtr(LEFT_INTAKE_PORT, false, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);
Motor right_intake_mtr(RIGHT_INTAKE_PORT, false, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);
Motor bottom_mtr(BOTTOM_PORT, false, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
Motor top_mtr(TOP_PORT, true, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);


void modified_initialize()
{
  pros::lcd::initialize();
  pros::lcd::print(0, "initialize");
  pros::lcd::print(3, "GOOD LUCK");

  inertial_initialize();
  tracker_initialize();
  encoder_initialize();
  //ultrasonic_initialize();
  odom_chassis_init();
}
