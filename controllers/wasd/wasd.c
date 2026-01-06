/*
 * File:          wasd.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/motor.h>, etc.
 */
#include <webots/robot.h>
#include <webots/keyboard.h>
#include <webots/motor.h>
/*
 * You may want to add macros here.
 */
#define TIME_STEP 64

/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
int main(int argc, char **argv) {
  /* necessary to initialize webots stuff */
  wb_robot_init();
  
  wb_keyboard_enable(TIME_STEP);
  
  WbDeviceTag shoulder_pan = wb_robot_get_device("shoulder_pan_joint");
  WbDeviceTag shoulder_lift = wb_robot_get_device("shoulder_lift_joint");
  WbDeviceTag elbow = wb_robot_get_device("elbow_joint");
  
  wb_motor_set_position(shoulder_pan, INFINITY);
  wb_motor_set_position(shoulder_lift, INFINITY);
  wb_motor_set_position(elbow, INFINITY);

  wb_motor_set_velocity(shoulder_pan, 0.0);
  wb_motor_set_velocity(shoulder_lift, 0.0);
  wb_motor_set_velocity(elbow, 0.0);

  while (wb_robot_step(TIME_STEP) != -1) 
  {
     int key = wb_keyboard_get_key();

    // Reset velocities each loop
    wb_motor_set_velocity(shoulder_pan, 0.0);
    wb_motor_set_velocity(shoulder_lift, 0.0);
    wb_motor_set_velocity(elbow, 0.0);

    switch (key) {
      case 'W': // Shoulder lift up
        wb_motor_set_velocity(shoulder_lift, 1.0);
        break;
      case 'S': // Shoulder lift down
        wb_motor_set_velocity(shoulder_lift, -1.0);
        break;
      case 'A': // Elbow bend
        wb_motor_set_velocity(elbow, 1.0);
        break;
      case 'D': // Elbow extend
        wb_motor_set_velocity(elbow, -1.0);
        break;
      case 'E': // Elbow extend
        wb_motor_set_velocity(shoulder_pan, 1.0);
        break;
      case 'Q': // Elbow extend
        wb_motor_set_velocity(shoulder_pan, -1.0);
        break;
    }
  };

  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}
