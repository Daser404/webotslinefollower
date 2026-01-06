/*
 * File:          line_follower.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/motor.h>, etc.
 */
#include <stdio.h>
#include <webots/robot.h>
#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <webots/accelerometer.h> 
#include <webots/gyro.h>


/*
 * You may want to add macros here.
 */
#define TIME_STEP 64
#define NUM_SENSORS 3
#define MAX_SPEED 6.28
/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
 void soft_left(WbDeviceTag left_motor,WbDeviceTag right_motor);
 void soft_right(WbDeviceTag left_motor,WbDeviceTag right_motor);
 
 
 
int main(int argc, char **argv) {
  /* necessary to initialize webots stuff */
  wb_robot_init();
  
  const char *gs_names[NUM_SENSORS] = {"gs0", "gs1", "gs2"}; WbDeviceTag gs[NUM_SENSORS];
  
  for (int i = 0; i < NUM_SENSORS; i++) 
  { 
  gs[i] = wb_robot_get_device(gs_names[i]); 
  wb_distance_sensor_enable(gs[i], TIME_STEP); 
  }
  
  static WbDeviceTag left_motor, right_motor;

  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY); 
  wb_motor_set_position(right_motor, INFINITY);
  //wb_motor_set_velocity(left_motor, 3.14);
  //wb_motor_set_velocity(right_motor, 3.14);
  
  WbDeviceTag acc = wb_robot_get_device("accelerometer");
  WbDeviceTag gyro = wb_robot_get_device("gyro"); 
  wb_accelerometer_enable(acc, TIME_STEP); 
  wb_gyro_enable(gyro, TIME_STEP);


  int prev = 0;
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
     
     double gs_value[3] = {0};
     for (int i = 0; i < NUM_SENSORS; i++) 
     gs_value[i] = wb_distance_sensor_get_value(gs[i]);
     
    wb_motor_set_velocity(left_motor, 3.14);
    wb_motor_set_velocity(right_motor, 3.14);
    
    if(gs_value[0] - gs_value[2] < -100)
    {
    soft_left(left_motor,right_motor);
    prev=1;
    }
    if(gs_value[0] - gs_value[2] > 100)
    {
    soft_right(left_motor,right_motor);
    prev=2;
    }
    if(gs_value[0]+gs_value[1]+gs_value[2]>2100)
    {
      if (prev==1)
      {
       wb_motor_set_velocity(left_motor, -3.14);
    wb_motor_set_velocity(right_motor, 3.14);
      }
      if(prev==2)
      {
       wb_motor_set_velocity(left_motor, 3.14);
    wb_motor_set_velocity(right_motor, -3.14);
      
      }
    }
    for (int i = 0; i < NUM_SENSORS; i++) 
    { 
    printf("gs%d = %f ", i, gs_value[i]);
    } 
    printf("\n\n");
    /* Process sensor data here */

    /*
     * Enter here functions to send actuator commands, like:
     * wb_motor_set_position(my_actuator, 10.0);
     */
     
     const double *a = wb_accelerometer_get_values(acc); 
     const double *g = wb_gyro_get_values(gyro); 
     printf("Acc: %f %f %f\n", a[0], a[1], a[2]); 
     printf("Gyro: %f %f %f\n", g[0], g[1], g[2]);
     printf("\n");printf("\n");printf("\n");
  };

  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}
 void soft_left(WbDeviceTag left_motor,WbDeviceTag right_motor)
 {
 wb_motor_set_velocity(left_motor, 0);
    wb_motor_set_velocity(right_motor, 1.14);
 }
 void soft_right(WbDeviceTag left_motor,WbDeviceTag right_motor)
 {
 wb_motor_set_velocity(left_motor, 1.14);
    wb_motor_set_velocity(right_motor, 0);
 }
