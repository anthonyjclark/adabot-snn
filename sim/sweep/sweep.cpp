
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

  // Simulation parameters
  if (argc < 6) {
    cerr << "Expected the following arguments:\n"
            "  <duration>\n"
            "  <left wheel speed>\n"
            "  <right wheel speed>\n"
            "  <strut extension percent>\n"
            "  <friction coefficient>";
    return -1;
  }

  const double duration = atof(argv[1]);
  const double left_wheel_speed = atof(argv[2]);
  const double right_wheel_speed = atof(argv[3]);
  const double strut_extension_percent = atof(argv[4]);
  const double friction_coeff = atof(argv[5]);

  cerr << "# simulation duration (s)     = " << duration
       << "\n# left wheel speed (rad/s)   = " << left_wheel_speed
       << "\n# right wheel speed (rad/s)  = " << right_wheel_speed
       << "\n# strut extension (%)        = " << strut_extension_percent
       << "\n# friction coefficient       = " << friction_coeff;

  constexpr double wheel_base = 18_cm;
  constexpr double track_width = 20_cm;
  constexpr double chassis_height = 3_cm;
  constexpr double wheel_radius = 3_cm;
  constexpr double wheel_thickness = 1.5_cm;
  constexpr double strut_radius = 0.5_cm;
  constexpr long strut_count = 5;

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

// #ifdef VISUALIZE
//   bool update_target = true;
//   adabot.logger.add_sphere(
//     "target", 5_cm * adabot.VIS_SCALE, { 0.1, 0.1, 0.1, 1.0 });
// #endif

  double strut_extension = wheel_radius * strut_extension_percent;

  cerr << "dlon,dlat,dyaw,left,right,strut\n";

  long time_ms = 0;
  long time_stop_ms = duration * 1000;

  auto chassis_transform = adabot.chassis->getTransform();
  auto yaw = chassis_transform.rotation().eulerAngles(2, 0, 2).y();
  auto pos = chassis_transform.translation();

  adabot.left_speed = left_wheel_speed;
  adabot.right_speed = right_wheel_speed;
  adabot.strut_extension = strut_extension;

  // Step forward in time and then measure changes
  while (time_ms < time_stop_ms) {
    adabot.step();
    time_ms += TIME_STEP_MS;
  }

  // Get dlon, dlat, and dyaw

  auto new_chassis_transform = adabot.chassis->getTransform();
  auto new_yaw = new_chassis_transform.rotation().eulerAngles(2, 0, 2).y();
  auto new_pos = new_chassis_transform.translation();

  double dyaw = new_yaw - yaw;
  if (dyaw > 1_pi) {
    dyaw -= 2_pi;
  } else if (dyaw < -1_pi) {
    dyaw += 2_pi;
  }

  // Get angle and distance to new position
  auto local_to_global = AngleAxisd(yaw, Vector3d(0, 1, 0));

  auto Va = local_to_global * Vector3d(1, 0, 0);
  auto Vb = new_pos - pos;
  auto Vn = Vector3d(0, 1, 0);

  double angle_to_new_pos = std::atan2(Va.cross(Vb).dot(Vn), Va.dot(Vb));
  double dist_traveled = (pos - new_pos).norm();

  double dlon = dist_traveled * cos(angle_to_new_pos);
  double dlat = dist_traveled * sin(angle_to_new_pos);

  // cerr << "dlon,dlat,dyaw,left,right,strut\n";
  cerr << dlon << ","
       << dlat << ","
       << dyaw << ","
       << left_wheel_speed << ","
       << right_wheel_speed << ","
       << strut_extension
       << "\n";

  yaw = new_yaw;

  adabot.log(cout);

  return 0;
}
