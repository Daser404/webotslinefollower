#include <webots/robot.h>
#include <webots/inverse_kinematics.h>
#include <webots/supervisor.h>
#include <webots/motor.h>

#define TIME_STEP 32

int main() {
  wb_robot_init();

  // Get the IK device
  WbDeviceTag ik = wb_robot_get_device("ik");

  // Enable the IK solver
  wb_inverse_kinematics_enable(ik, TIME_STEP);

  // Get the target node (defined in the .wbt world)
  WbNodeRef target = wb_supervisor_node_get_from_def("TARGET");
  WbFieldRef target_translation = wb_supervisor_node_get_field(target, "translation");
  WbFieldRef target_rotation = wb_supervisor_node_get_field(target, "rotation");

  // Example Cartesian target
  const double pos[3] = {0.4, 0.2, 0.3};   // X Y Z in meters
  const double rot[4] = {1, 0, 0, 0};      // axis-angle rotation (no rotation)

  // Set the target position
  wb_supervisor_field_set_sf_vec3f(target_translation, pos);

  // Set the target orientation
  wb_supervisor_field_set_sf_rotation(target_rotation, rot);

  // Let Webots compute IK and move the arm
  while (wb_robot_step(TIME_STEP) != -1) {
    // Nothing else needed — IK runs automatically
  }

  wb_robot_cleanup();
  return 0;
}
