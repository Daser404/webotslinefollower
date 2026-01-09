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
#include <webots/distance_sensor.h>
#include <math.h>


/*
 * You may want to add macros here.
 */
#define TIME_STEP 64
#define MAX_SPEED 6.28
#define STUCK_THREASHOLD 30
#define SEARCH_CONSTANT 0.0
/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */

 
int main(int argc, char **argv) {
  /* necessary to initialize webots stuff */
  wb_robot_init();
  
  ///difine the name of the sensors
  const char *gs_names[3] = {"gs0", "gs1", "gs2"}; 
  
  ///assing the ground sensors to device variables and enable the ground sensors
  WbDeviceTag gs[3];
  
  for (int i = 0; i < 3; i++) 
  { 
    gs[i] = wb_robot_get_device(gs_names[i]); 
    wb_distance_sensor_enable(gs[i], TIME_STEP); 
  }
  double gs_value[3] = {0};
  
   ///assing the proximity sensors to device variables and enable the proximity sensors
  WbDeviceTag prox_sensors[8];
  char prox_sensor_name[50];
  for ( int ind = 0; ind < 8; ind++){
    sprintf(prox_sensor_name, "ps%d", ind);
    prox_sensors[ind] = wb_robot_get_device(prox_sensor_name);
    wb_distance_sensor_enable(prox_sensors[ind], TIME_STEP);
  }
  double ps_value[8] = {0};
  
  ///assing the motors to device variables and enable the motors
  static WbDeviceTag left_motor, right_motor;

  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY); 
  wb_motor_set_position(right_motor, INFINITY);
  
  ///assing and enable the accelerometer and gyroscope
  WbDeviceTag acc = wb_robot_get_device("accelerometer");
  WbDeviceTag gyro = wb_robot_get_device("gyro"); 
  wb_accelerometer_enable(acc, TIME_STEP); 
  wb_gyro_enable(gyro, TIME_STEP);
  
  ///save the last error 
  ///used for when the robot looses the track / is with all sensors on the line
  static double last_error = 0;
  static bool turning = 0;
  double velocity = 0; //m/s
  static int state = 0;//0 = searching for line , 1 = avoiding obstacle, 2 = following line
  static double search = SEARCH_CONSTANT; // used for searching the line in a spiral that gets bigger
  int stuck_ellapsed = 0;
  double base_speed = 1.2;
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
     
    
    
    double left_speed;
    double right_speed;   
    
     
     
     
     
     
     
     ///read and store the ground sensor readings in an array
     for (int i = 0; i < 3; i++) 
     gs_value[i] = wb_distance_sensor_get_value(gs[i]);
     
    
     ///invert the sensor reading to help with readability
     double sL = 1000.0 - gs_value[0];
     double sC = 1000.0 - gs_value[1]; 
     double sR = 1000.0 - gs_value[2]; 
     
     /// Compute weighted line position 
     double sum = sL + sC + sR; 
     
     if (sum < 1e-6) sum = 1e-6; ///avoid dividing by 0
     
     double line_position = (sL * -1.0 + sC * 0.0 + sR * 1.0) / sum;
     
    double error = line_position;
    
    ///variables that tell us if the robot is off trock or with all three sensors on the line
    bool all_black = (sL > 500 && sC > 500 && sR > 500);
    bool off_track = (sL < 200 && sC < 200 && sR < 200);
    if(all_black==0&&off_track==0)
    stuck_ellapsed = 0;
    
    double K = 2; // tuning constant
    double rotation = K * error;
    
     
    left_speed = base_speed + rotation;
    right_speed = base_speed - rotation; 
    
    ///if the robot is stuck with all three sensors on black we enter this case 
    if (all_black) 
    {
      stuck_ellapsed ++;//count how many cycles the robot has been stuck
      
      if(stuck_ellapsed>=STUCK_THREASHOLD) // if the robot has been stuck for more than STUCK_THREASHOLD cycles we perform a tank turn in the last direction performed
      {
        if (last_error > 0) 
         {
           // turn right 
           left_speed = base_speed;
           right_speed = base_speed *0.8;
         }
     
       else
       {
         // turn left
         left_speed = base_speed *0.8;
         right_speed = base_speed ;
       }
     }
    }
    
    
    ///if the robot is stuck with all three sensors on white we enter this case
    if (off_track) 
    {
      stuck_ellapsed ++;//count how many frames the robot has been stuck
      
      if(stuck_ellapsed>=STUCK_THREASHOLD)// if the robot has been stuck for more than 15 frames we perform a tank turn in the last direction performed
      {
        if (last_error == 0)//searching for the line (only at the beginning) 
        {
         // forward
         left_speed = base_speed;
         right_speed = base_speed;
        }
       else if (last_error > 0) 
        {
         state = 0;
         //turn right 
         left_speed = base_speed;
         right_speed = base_speed * search;///-0.8;
         if(search < 0.9)
         search += 0.008;
        }
       else
        {
          state = 0;
         //turn left
         left_speed = base_speed * search;///-0.8;
         right_speed = base_speed;
         if(search < 0.9)
         search += 0.008;
        }
      }
    }
    
    
    ///reading the sensor values from the proximity sensors
     for (int i = 0; i < 8; i++) 
     ps_value[i] = wb_distance_sensor_get_value(prox_sensors[i]);
    
    
    /// check where we have obstacles
    bool front_wall_right = ps_value[0] > 120.0; 
    bool front_wall_left = ps_value[7] > 120.0;
    
    bool left_wall  = ps_value[5] > 80.0;
    bool right_wall  = ps_value[2] > 80.0;
    
    
    ///if the object is more to the left we turn right on the spot until it is sensed by the left sensor
    if (front_wall_left || turning == 1) 
    {    
      state = 1; // enter the obstacle avoidance state
      stuck_ellapsed = 0;
      
      // we start turning right
      turning = 1; 
      left_speed = base_speed;
      right_speed = -base_speed;
    } 
    ///when the left sensor reads the object we start going forward with a slight left turn
    if (left_wall && off_track && state == 1)
    {
      stuck_ellapsed = 0;
      turning = 0;/// stop turning
      
      //start going forward with a slight left turn
      left_speed  = base_speed*0.7;
      right_speed = base_speed;
    } 
    
    ///if the object is more to the right we turn left on the spot until it is sensed by the left right
    if (front_wall_right || turning == -1) 
    {    
      state = 1; // enter the obstacle avoidance state
      stuck_ellapsed = 0;
      
      // we start turning left
      turning = -1;
      left_speed = -base_speed;
      right_speed = +base_speed;
    } 
    
    ///when the right sensor reads the object we start going forward with a slight right turn
    if (right_wall && off_track && state == 1)
    {
      stuck_ellapsed = 0;
      turning = 0;/// stop turning
      
      //start going forward with a slight right turn
      left_speed  = base_speed;
      right_speed = base_speed*0.7;
    } 
    
    ///if the robot reads the line
    ///stop turning
    if (off_track == 0)
    {
      state = 2;///enter the line following state
      turning = 0; ///stop turning
      search = SEARCH_CONSTANT;///reset the search spiral to small
    }

    
    
    ///change the motor speeds with the calculated values
    wb_motor_set_velocity(left_motor, left_speed); 
    wb_motor_set_velocity(right_motor, right_speed);
    
    ///change the last_error for the next cycle
    if(error > 0.3||error < -0.3)
    last_error = error;
    
    ///print the ground sensor reeadings
    for (int i = 0; i < 3; i++) 
    { 
      printf("gs%d = %f ", i, gs_value[i]);
    }
    for (int i = 0; i < 8; i++) 
    { 
      printf("ps%d = %f ", i, ps_value[i]);
    }
    
    printf("\nlast error = %f     error = %f       off_track = %d     all_black = %d      stuck_ellapsed = %d\n",last_error,error, off_track, all_black,stuck_ellapsed);
     
    ///print the accelerometer and gyroscope sensor reeadings 
    const double *a = wb_accelerometer_get_values(acc); 
    const double *g = wb_gyro_get_values(gyro); 
    velocity+=a[0]/15.625;
    printf("Acc: %f %f %f       speed(cm/s): %lf\n", a[0], a[1], a[2], velocity*100); 
    printf("Gyro: %f %f %f       state: %d\n", g[0], g[1], g[2], state);
    printf("\n");printf("\n");printf("\n");
    
  };

  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}

