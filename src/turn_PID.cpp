#include "okapi/api.hpp"
#include "inertial.hpp"
using namespace okapi;

//Creates a constant for wheel diameter
const double wheel_diam = 4.0;
//Creates a constant for pi
const double drive_pi = 3.14159265359;
//Calculates a constant wheel circumference using diameter and pi
const double wheel_circ = wheel_diam * drive_pi;
//Encoder degrees in circumference
const double degrees_per_circ = 360.0;
//Encoder degrees per inch
const double degrees_per_inch = degrees_per_circ / wheel_circ;

//Turn x degrees at y speed
void gyro_turn(okapi::ChassisController& chassis, QAngle angle, double max_speed, double min_speed, double kp, double ki, double kd, double epsilon)
{

    //Chassis arcade takes values from -1 to 1 so this line allows a value from -100 to 100
    //if (max_speed > 60) max_speed = 60;
    max_speed = max_speed / 100;
    if (min_speed < 17.5) min_speed = 17.5;
    min_speed = min_speed / 100;

    //Sets the encoder units to se degrees instead of ticks
    chassis.getModel()->setEncoderUnits(AbstractMotor::encoderUnits::degrees);
    //Setting the Proportional,Integral,Differential constants (P.I.D.)
    double turn_kp = kp;//0.011;
    double turn_ki = ki;//0.000;
    double turn_kd = kd;//0.000;
    //Creates a constant for allowable error before stopping
    double turn_epsilon = epsilon;//2.0;
    //Creates a maximum speed for velocity adjustment so that the robot will accelerate smoothly
    //and have no jerk at the beggining
    const double turn_maximum_vel_adj = 5.0;
    //States the window in which the integral will be activated
    const double turn_integral_limit = 50.0;

    //Converts the Qangle to degrees
    double distance_in_degrees = angle.convert(degree);
    //Draws starting position from the gyro
    double start_pos_in_degrees = inertial_get_value();
    //Calculates current position based on start position
    double current_pos_in_degrees = inertial_get_value() - start_pos_in_degrees;
    //Sets the initial last pos to zero
    double last_current_pos_in_degrees = current_pos_in_degrees;
    //Sets the integral to zero so that additions can be made later
    double turn_integral = 0.0;


    double second_last_turn_error = 9999.9;

    double third_last_turn_error = 9999.9;

    double fourth_last_turn_error = 9999.9;

    //Defines the initial drive error
    double turn_error = distance_in_degrees - current_pos_in_degrees;

    //Sets last error to zero before turning starts
    double turn_last_error = turn_error;

    double last_three_turn_derivatives = 9999.9;



    //ACCELERATION FOR LATER
    //Sets the first speed to zero
    //double last_speed = 0.0;

    //Keep turning while the robot hasn't reached its target distance
    while ((fabs(last_three_turn_derivatives) > turn_epsilon) || (fabs(turn_error) > turn_epsilon*2.0))
    {
        // ******************************************************************************************************************************* //
        //  This code uses proportional , differential, and integral constants to calculate the best speed to reach the desired distance   //
        // ******************************************************************************************************************************* //

        // This code accounts for the fact that when the robot turns more than 360 degrees in one direction the gyroscope will still read proper values
        if(((current_pos_in_degrees + 180) < last_current_pos_in_degrees) && (distance_in_degrees > last_current_pos_in_degrees))
        {
          distance_in_degrees = distance_in_degrees - 360;
        }

        else if(((current_pos_in_degrees -180)> last_current_pos_in_degrees) && (distance_in_degrees < last_current_pos_in_degrees))
        {
          distance_in_degrees = distance_in_degrees + 360;
        }

        last_current_pos_in_degrees = current_pos_in_degrees;


        //Calculate distance left to turn
        turn_error = distance_in_degrees - current_pos_in_degrees;

        printf("current: %f  error: %f ", current_pos_in_degrees, turn_error);

        //Calculates the derivative
        double turn_derivative = turn_last_error - turn_error;

        fourth_last_turn_error = third_last_turn_error;

        third_last_turn_error = second_last_turn_error;

        second_last_turn_error = turn_last_error;

        //Sets a new last error
        turn_last_error = turn_error;

        double third_last_turn_derivative = fourth_last_turn_error - third_last_turn_error;

        double second_last_turn_derivative = third_last_turn_error - second_last_turn_error;

        double last_turn_derivative = second_last_turn_error - turn_last_error;

        last_three_turn_derivatives = last_turn_derivative + second_last_turn_derivative + third_last_turn_derivative;


        //Determines when the integral will start being used
        if(turn_ki != 0)
        {
            if(fabs(turn_error) < turn_integral_limit)
            {
                turn_integral = turn_integral + turn_error;
            }
            else
            {
                turn_integral = 0;
            }
        }
        else
        if((turn_integral > 0) && (turn_error < 0))
        {
          turn_integral = turn_integral * -1;
        }
        else if((turn_integral < 0) && (turn_error > 0))
        {
          turn_integral = turn_integral * -1;
        }

        //Calculate speed to be turned at using kp,ki,kd
        double speed = turn_error * turn_kp + turn_integral * turn_ki + turn_derivative * turn_kd;
        printf("stop: %f  Speed: %f  (p,i,d): (%f,%f,%f) ",last_three_turn_derivatives,speed,turn_error*turn_kp,turn_integral*turn_ki,turn_derivative*turn_kd);

        //Removes impossible speeds by setting the speed down to a possible one
        if(speed > max_speed)
        {
            speed = max_speed;
        }

        if(speed < max_speed * -1)
        {
            speed = max_speed * -1;
        }

        if ((speed > 0) && (speed < min_speed))
        {
          speed = min_speed;
        }
        if ((speed < 0) && (speed > -min_speed))
        {
          speed = -min_speed;
        }

        printf("adj speed: %f\n",speed);

        //Setting the desired speed in a percent form and waiting 10 milliseconds
        //chassis.getModel()->arcade(0.0, speed);
        chassis.getModel()->arcade(0.0, speed);
        pros::delay(33);

        //Calculates current position based on start position after small movement
        current_pos_in_degrees =  inertial_get_value() - start_pos_in_degrees;
    }

    //Stops the motors
    chassis.getModel()->stop();
}


void gyro_turn_to(okapi::ChassisController& chassis, QAngle angle, double max_speed, double min_speed)
{
    double new_turn = angle.convert(degree) - inertial_get_value();
    QAngle new_angle = new_turn * degree;
    //gyro_turn(chassis, new_angle, max_speed, 17.5);
}
