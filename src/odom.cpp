#include "initialize.hpp"
#include "okapi/api.hpp"
#include "position_tracker.hpp"
#include "inertial.hpp"
#include "firmware.hpp"
#include "ultrasonic.hpp"
using namespace okapi;


#define RIGHT_REAR_WHEEL_PORT 20
#define LEFT_REAR_WHEEL_PORT 11
#define RIGHT_FRONT_WHEEL_PORT 10
#define LEFT_FRONT_WHEEL_PORT 1

std::shared_ptr<OdomChassisController> chassis = NULL;
//std::shared_ptr<ChassisController> chassis = NULL;

void odom_chassis_init()
{
    chassis = ChassisControllerBuilder()
      .withMotors( LEFT_FRONT_WHEEL_PORT, LEFT_REAR_WHEEL_PORT, -RIGHT_FRONT_WHEEL_PORT, -RIGHT_REAR_WHEEL_PORT ) // left motor is 1, right motor is 2 (reversed)
      //.withMotors( LEFT_FRONT_WHEEL_PORT, -RIGHT_FRONT_WHEEL_PORT )
      // .withGains(
      //     {0.001, 0, 0.0001}, // distance controller gains
      //     {0.001, 0, 0.0001}, // turn controller gains
      //     {0.001, 0, 0.0001}  // angle controller gains (helps drive straight)
      // )
      .withSensors(
          *shaft_enc_l,
          *shaft_enc_r//,
          //*shaft_enc_m
      )
    //
    //     // ADIEncoder{'C', 'D'}, // left encoder in ADI ports A & B
    //     // ADIEncoder{'A', 'B'},  // right encoder in ADI ports C & D (reversed)
    //     // ADIEncoder{'E', 'F'}  // middle encoder in ADI ports E & F
    //
    //     //ADIEncoder{'G', 'H'}, // left encoder in ADI ports A & B
    //     //ADIEncoder{'E', 'F', true},  // right encoder in ADI ports C & D (reversed)
    //     //ADIEncoder{'C', 'D', true}  // middle encoder in ADI ports E & F
    // green gearset, tracking wheel diameter (2.75 in), track (7 in), and TPR (360)
    // 1 inch middle encoder distance, and 2.75 inch middle wheel diameter

    //2.75 = wheel diameter of tracking wheels
    //13.5 = distance between large wheels
    //0 = middle encoder distance
    //2.75 = diameter of middle tracking wheel

    //home robot
    //.withDimensions(AbstractMotor::gearset::green, {{3.25_in, 13.5_in, 0_in, 3.25_in}, quadEncoderTPR})
    //.withOdometry() // use the same scales as the chassis (above)
    //.buildOdometry(); // build an odometry chassis
    .withDimensions(AbstractMotor::gearset::green, {{4_in, 13.5_in}, imev5GreenTPR})
    .withMaxVelocity(50)
    .withOdometry({{3.25_in, 13.5_in},quadEncoderTPR})
    .buildOdometry();
    //.build();


    // .withDimensions(AbstractMotor::gearset::green, {{2.75_in, 13_in, 3_in, 2.75_in}, quadEncoderTPR})
    // .withOdometry() // use the same scales as the chassis (above)
    // .buildOdometry(); // build an odometry chassis
}

void drive_to_point(QLength x, QLength y)
{
  chassis->setState({0_in, 0_in, 0_deg});
  chassis->driveToPoint({-12.0_in, 0_in});
  //chassis->driveToPoint({x, y});
  //chassis->turnAngle(45_deg);
  //chassis->moveDistance(1.0_ft);
}
