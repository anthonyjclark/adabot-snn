
#include <iostream>
using std::cerr;
using std::cout;
using std::endl;

#include <sstream>
using std::istringstream;

#include "../ugv.hpp"


int main(int argc, char const* argv[])
{
  constexpr double TIME_STEP = 0.005;
  constexpr long TIME_STEP_MS = TIME_STEP * 1000;
  constexpr double TIME_STOP_MS = 60 * 1000;

  // Simulation parameters
  if (argc < 7) {
    cerr << "Expected the following arguments:\n"
            "  <number of struts>\n"
            "  <FSM turning degrees>\n"
            "  <maximum wheel speed>\n"
            "  <inside turn factor>\n"
            "  <strut extension percent>\n"
            "  <friction coefficient>";
    return -1;
  }

  const long strut_count = atoi(argv[1]);
  const double fsm_deg = atof(argv[2]);
  const double wheel_speed = atof(argv[3]);
  const double turn_factor = atof(argv[4]);
  const double strut_extension_percent = atof(argv[5]);
  const double friction_coeff = atof(argv[6]);

  cerr << "# number of struts            = " << strut_count
       << "\n# FSM turning (degrees)       = " << fsm_deg
       << "\n# maximum wheel speed (rad/s) = " << wheel_speed
       << "\n# inside wheel turning factor = " << turn_factor
       << "\n# strut extension (%)         = " << strut_extension_percent
       << "\n# friction coefficient        = " << friction_coeff;

  const double fsm_rad = fsm_deg * 0.0174533;

  constexpr double wheel_base = 18_cm;
  constexpr double track_width = 20_cm;
  constexpr double chassis_height = 3_cm;
  constexpr double wheel_radius = 3_cm;
  constexpr double wheel_thickness = 1.5_cm;
  constexpr double strut_radius = 0.5_cm;

  cerr << "\n# wheel base (m)              = " << wheel_base
       << "\n# track width (m)             = " << track_width
       << "\n# wheel radius (m)            = " << wheel_radius << "\n";

  UGV adabot(wheel_base,
             track_width,
             chassis_height,
             wheel_radius,
             wheel_thickness,
             strut_count,
             strut_radius,
             TIME_STEP,
             friction_coeff);

  // When to print ANN training data
  constexpr long DATA_STEP_MS = 100;
  long next_data_update_time = DATA_STEP_MS;

  // When to update control parameters
  const long CONTROL_STEP_MS = 100;
  long next_control_update_time = 0;

  double dist = 50_cm;
  vector<Vector3d> targets{
    {  dist, 0,     0 },
    {     0, 0,  dist },
    {  dist, 0,  dist },
    { -dist, 0,  dist },
    {     0, 0, -dist },
    { -dist, 0, -dist },
    { -dist, 0,     0 },
    {  dist, 0, -dist },
    {     0, 0,     0 },
  };

  size_t target_idx = 0;
  double target_dist = 0.0;
  double target_angle = 0.0;

#ifdef VISUALIZE
  bool update_target = true;
  adabot.logger.add_sphere(
    "target", 5_cm * adabot.VIS_SCALE, { 0.1, 0.1, 0.1, 1.0 });
#endif

  double left_speed = 0;
  double right_speed = 0;
  double strut_extension = wheel_radius * strut_extension_percent;

  // cerr << "time,x,y,z,target_dist,target_angle,left_speed,right_speed,strut\n"
  //         "0,0,0,0," << dist << ",0,0,0,0\n";

  cerr << "time,dx,dz,left,right,strut\n"
          "0,0,0,0,0,0\n";

  long time_ms = 0;
  double x = 0.0;
  double z = 0.0;

  while (time_ms <= TIME_STOP_MS) {

    //
    // Update Adabot control parameters
    //
    if (next_control_update_time <= time_ms) {
      next_control_update_time += CONTROL_STEP_MS;

      // Get angle and distance to target
      auto chassis_transform = adabot.chassis->getTransform();

      auto chassis_yaw = chassis_transform.rotation().eulerAngles(2, 0, 2).y();
      auto chassis_pos = chassis_transform.translation();

      auto local_to_global = AngleAxisd(chassis_yaw, Vector3d(0, 1, 0));

      auto Va = local_to_global * Vector3d(1, 0, 0);
      auto Vb = targets[target_idx] - chassis_pos;
      auto Vn = Vector3d(0, 1, 0);

      target_angle = std::atan2(Va.cross(Vb).dot(Vn), Va.dot(Vb));
      target_dist = (chassis_pos - targets[target_idx]).norm();

      // Update target if necessary
      if (target_dist < 10_cm) {
        if (++target_idx >= targets.size()) {
          target_idx = 0;
          strut_extension *= 0.5;
        }
#ifdef VISUALIZE
        update_target = true;
#endif
      }

      // Navigate to target
      if (-fsm_rad < target_angle && target_angle < fsm_rad) {
        left_speed = -wheel_speed;
        right_speed = -wheel_speed;
      } else if (target_angle < -fsm_rad) {
        left_speed = -wheel_speed * turn_factor;
        right_speed = wheel_speed;
      } else {
        left_speed = wheel_speed;
        right_speed = -wheel_speed * turn_factor;
      }

      adabot.left_speed = left_speed;
      adabot.right_speed = right_speed;
      adabot.strut_extension = strut_extension;
    }


    adabot.step();
    time_ms += TIME_STEP_MS;


    //
    // Output ANN training data
    //
    if (next_data_update_time <= time_ms) {
      next_data_update_time += DATA_STEP_MS;

      auto chassis_transform = adabot.chassis->getTransform();
      auto chassis_translation = chassis_transform.translation();

      // cerr << "time,dx,dz,left,right,strut\n"

      double new_x = chassis_translation.x();
      double new_z = chassis_translation.z();

      cerr << adabot.world->getTime() << ","
           << new_x - x << ","
           // << chassis_translation.y() << ","
           << new_z - z << ","
           // << target_dist << ","
           // << target_angle << ","
           << left_speed << ","
           << right_speed << ","
           << strut_extension << "\n";

      x = new_x;
      z = new_z;
    }


    //
    // Update target visualization when adabot visualization is updated
    //
#ifdef VISUALIZE
    if (adabot.vis_updated && update_target) {
      update_target = false;
      adabot.vis_updated = false;
      adabot.logger.add_to_frame("target",
                                 targets[target_idx].x() * adabot.VIS_SCALE,
                                 targets[target_idx].y() * adabot.VIS_SCALE,
                                 targets[target_idx].z() * adabot.VIS_SCALE,
                                 0, 0, 0, 1);
    }
#endif

  }
  adabot.log(cout);

  return 0;
}
