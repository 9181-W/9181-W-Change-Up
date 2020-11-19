//Includes
#include "okapi/api.hpp"
#include "inertial.hpp"
#include "firmware.hpp"
// Include additional mathematical operations
#include <cmath>
using namespace okapi;

//Creates a constant for wheel diameter
const double wheel_diam = 2.75;
//Creates a constant for pi
const double drive_pi = 3.14159265359;
//Calculates a constant wheel circumference using diameter and pi
const double wheel_circ = wheel_diam * drive_pi;
//Encoder degrees in circumference
const double degrees_per_circ = 360.0;
//Encoder degrees per inch
const double degrees_per_inch = degrees_per_circ / wheel_circ;

//a variable to easily take measurements of time in a loop
int millisecondStepTime = 15;

//This is something you should utilize to convert the horizontal encoder distance to match the vertical encoder distance
//Grab a ruler, and push the robot 20 inches straight forward, then record the average vertical value (Example: 51269)
//Reset, Grab a ruler, and push the robot 20 inches to the right, then record the horizontal encoder value (Example: 10532)
//You will need to convert it such that the horizontal matches for the same distance, so the multiplier will be horizontal(10532)/vertical(51269)
//And that number will be the ratio (10532/51269 = 0.2054262809885116)
double verticalToHorizontalRatio = 1.0000000;

//Because we can average the vertical encoders while turning, it will not show a change
//But for the horizontal encoders, it will assume the robot is laterally strafing, which is not correct
//We fix this by applying a multiplier compensation such that we will negate it using the inertial sensor
//Reset your robot, make sure everything is calibrated
//Rotate your robot such that the inertial sensor reads 3600 degrees (i.e. rotate the robot 10 times for the sake of accuracy)
//Your horizontal encoder should have a value (Example: 231809)
//Divide that result by 3600 (231809/3600 = 64.3913888), and that number will be the value used
//-1.68?
double rotateNegateHorizontalRatio = -147.489/3570.3;

//This is utilized to convert the ticks moved to actual inches
//Push the robot 20 inches forward (without turning it or pushing it strafe-wise)
//Get the average reading of the vertical encoders (Example: 253053)
//Divide the reading by 20 (253053/20 = 12,652.65), and put the multipler in this area
//This will convert it to inches
//double posTranslationMultiplier = 1.0;

//Multipliers for correcting erroneous data from the Encoders
const double l_enc_err_multiplier = 1.0169;
const double r_enc_err_multiplier = 1.0239;
const double m_enc_err_multiplier = 1.01416667;

//Function to convert degrees to radians
double getRadians(int degrees)
{
  return (degrees * M_PI ) / 180 ;
}

//Creates a variable to contain the last value of the vertical and horizontal position and sets its initial value to 0.0
double lastVerticalPosLocal = 0.0;
double lastHorizontalPosLocal = 0.0;

//Creates a variable to contain the x position and y position and sets the initial values to 0.0
double XPos = 0.0;
double YPos = 0.0;

//creates variables to contain the x and y values of the tracker and sets their initial value to 0.0
double tracker_x_value = 0.0;
double tracker_y_value = 0.0;

//function to get the robots location
void tracker_location(void* param)
{
  while (true)
  {
    // Wait an amount of seconds, while returning the actual wait time to the variable "m"
    //When telling a computer to wait, it may have a load-amount that would result in it yielding earlier or later then the actual time, that's why we do this
    //ask the time
    uint32_t time_1 = pros::millis();
    pros::delay(millisecondStepTime);
    uint32_t time_2 = pros::millis();
    int m = time_2 - time_1;

    //Gets Values of Encoders
    //This gets the position of the sensors (this is local position)
    double leftEnc = ((shaft_enc_l->get() / degrees_per_inch) * l_enc_err_multiplier);
    double rightEnc = ((shaft_enc_r->get() / degrees_per_inch) * r_enc_err_multiplier);
    double horizEnc = ((shaft_enc_m->get() / degrees_per_inch) * m_enc_err_multiplier);

    // double leftEnc = shaft_enc_l->get();
    // double rightEnc = shaft_enc_r->get();
    // double horizEnc = shaft_enc_m->get();

    //double horizEnc = shaft_enc_m->get();

    // Get the rotation of the inertial sensor
    double RobotRotation = inertial_get_value();;
    /////////////////////////////////////////////////////////////////////////

    /////////////////////////////////////////////////////////////////////////
    //This is for the local position of the robot (THIS IS NOT WORLD POSITION)
    double VerticalPosLocal = (leftEnc + rightEnc) / 2.0;
    double HorizontalPosLocal = horizEnc + (RobotRotation * rotateNegateHorizontalRatio);
    /////////////////////////////////////////////////////////////////////////

    //Applies ratio
    HorizontalPosLocal *= verticalToHorizontalRatio;

    //By getting position - lastposition within 20 miliseconds, this will give us velocity
    //Similar to PID alrogithms, this converts position -> Velocity by gettings its derivative through the continuous loop
    double YV = VerticalPosLocal - lastVerticalPosLocal;
    double XV = HorizontalPosLocal - lastHorizontalPosLocal;

    //Do some relatively-complex math that converts local velocity to world-space velocity. The (m/milisecondStepTime) at the end is a multiplier
    //that would scale velocity more appropriately due to computer yields (Line 131 explains) If computer is told to wait 2 second and waits 0.75 instead,
    //the multiplier would then scale the velocity to be 0.375 its value (0.75/2 = 0.375)
    //double XVelocityWorldSpace = (XV * std::cos(getRadians(-RobotRotation)) - YV * std::sin(getRadians(-RobotRotation))) * ((m * 1.00) / (millisecondStepTime * 1.0));
	  //double YVelocityWorldSpace = (XV * std::sin(getRadians(RobotRotation)) - YV * std::cos(getRadians(RobotRotation))) * ((m * 1.00) / (millisecondStepTime * 1.0));
    //double XVelocityWorldSpace = (XV * std::cos(getRadians(-RobotRotation)) - YV * std::sin(getRadians(-RobotRotation)));
	  //double YVelocityWorldSpace = (XV * std::sin(getRadians(RobotRotation)) - YV * std::cos(getRadians(RobotRotation)));
    double XVelocityWorldSpace = XV * std::cos(getRadians(RobotRotation)) - YV * std::sin(getRadians(RobotRotation));
    double YVelocityWorldSpace = XV * std::sin(getRadians(RobotRotation)) + YV * std::cos(getRadians(RobotRotation));


    //This would give us the X and Y position in world space
    XPos += XVelocityWorldSpace;
    YPos += YVelocityWorldSpace;

    //This is applied to be utilized 20 miliseconds after
    lastVerticalPosLocal = VerticalPosLocal;
    lastHorizontalPosLocal = HorizontalPosLocal;

    //sets the x and y value to the x and y position
    tracker_x_value = XPos;
    tracker_y_value = YPos;

    //prints the x and y value to the brain
    pros::lcd::print(1,"Tracker X Val %f",tracker_x_value);
    pros::lcd::print(2,"Tracker Y Val %f",tracker_y_value);

    //prints the encoder values to the brain
    pros::lcd::print(3,"Left Encoder %f",leftEnc);
    pros::lcd::print(4,"Right Encoder %f",rightEnc);
    pros::lcd::print(5,"Middle Encoder %f",horizEnc);

    pros::delay(20);
  }
}

//Function to initialize a task that reads the tracker values
void tracker_initialize()
{
  //uses the built-in pros task creator to start a task
  pros::Task tracker_location_task (tracker_location, (void*)"PROSV5", TASK_PRIORITY_DEFAULT,
    TASK_STACK_DEPTH_DEFAULT, "Tracker Location Task");
}

//function that returns the x value of the robot
double tracker_get_x()
{
  return tracker_x_value;
}

//function that returns the y value of the robot
double tracker_get_y()
{
  return tracker_y_value;
}
