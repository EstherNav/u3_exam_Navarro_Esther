/*
 * File:          omni_Navarro_Esther.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/differential_wheels.h>, etc.
 */
#include <webots/robot.h>
#include <webots/robot.h>
#include <webots/position_sensor.h>
#include <webots/keyboard.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>

#include <stdio.h>
#include <math.h>

/*
 * You may want to add macros here.
 */
#define TIME_STEP 64
#define PI 3.141592
#define OBSTACLE_DIST 120.0
/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
 
 enum {
  GO,
  TURNRIGHT,
  TURNLEFT,
  FREEWAY,
  OBSTACLE,
  AUTONOMUS,
  MANUAL,
  LEFT,
  RIGTH
};

 int mode = AUTONOMUS;
double straightLineAngle;

int searchForObstacles(WbDeviceTag distance_sensor) {
  double distance_of_sensor = wb_distance_sensor_get_value(distance_sensor);
  if (distance_of_sensor > OBSTACLE_DIST)
    return FREEWAY;
  else
    return OBSTACLE;
}
 void fowardLinearly(WbDeviceTag *wheels, double velocity) {
  wb_motor_set_velocity(wheels[0], -6);
  wb_motor_set_velocity(wheels[1], 6);
  wb_motor_set_velocity(wheels[2], 0);
}

void backwardLinearly(WbDeviceTag *wheels){
  wb_motor_set_velocity(wheels[0], 6);
  wb_motor_set_velocity(wheels[1], -6);
  wb_motor_set_velocity(wheels[2], 0);
}

void rightLinearly(WbDeviceTag *wheels){
  wb_motor_set_velocity(wheels[0], 6);
  wb_motor_set_velocity(wheels[1], 0);
  wb_motor_set_velocity(wheels[2],-6);
}

void leftlinearly(WbDeviceTag *wheels){
  wb_motor_set_velocity(wheels[0],-6);
  wb_motor_set_velocity(wheels[1], 0);
  wb_motor_set_velocity(wheels[2], 6);
}

void wheelsTurnRight(WbDeviceTag *wheels){
  wb_motor_set_velocity(wheels[0], -6);
  wb_motor_set_velocity(wheels[1], -6);
  wb_motor_set_velocity(wheels[2], -6);
}

void wheelsTurnLeft(WbDeviceTag *wheels){
  wb_motor_set_velocity(wheels[0], 6);
  wb_motor_set_velocity(wheels[1], 6);
  wb_motor_set_velocity(wheels[2], 6);
}

void stopWheels(WbDeviceTag *wheels) {
  wb_motor_set_velocity(wheels[0], 0);
  wb_motor_set_velocity(wheels[1], 0);
  wb_motor_set_velocity(wheels[2], 0);
}

double getAngleRobot(WbDeviceTag pos_sensor) {
  printf("Angle calculation\n");
  double angle, rotationAngleW1;

  rotationAngleW1 = wb_position_sensor_get_value(pos_sensor);
  angle = fabs(rotationAngleW1- straightLineAngle);
  printf("Angle: %lf\n", angle);

  return angle;
}
float clearAngleRobot() {
  printf("Clearing angle\n"
 }
 
int main(int argc, char **argv)
{
  /* necessary to initialize webots stuff */
  wb_robot_init();

  /*
   * You should declare here WbDeviceTag variables for storing
   * robot devices like this:
   *  WbDeviceTag my_sensor = wb_robot_get_device("my_sensor");
   *  WbDeviceTag my_actuator = wb_robot_get_device("my_actuator");
   */

  /* main loop
   * Perform simulation steps of TIME_STEP milliseconds
   * and leave the loop when the simulation is over
   */
  while (wb_robot_step(TIME_STEP) != -1) {

    /*
     * Read the sensors :
     * Enter here functions to read sensor data, like:
     *  double val = wb_distance_sensor_get_value(my_sensor);
     */

    /* Process sensor data here */

    /*
     * Enter here functions to send actuator commands, like:
     * wb_differential_wheels_set_speed(100.0,100.0);
     */
  };

  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}
