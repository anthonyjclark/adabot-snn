
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

  // Update control and simulate before measuring change
  constexpr long UPDATE_STEP_MS = 100;
  long next_update_time_ms = 0;

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


  // Direction controller
  struct State {
    std::string name;
    double left_speed, right_speed;
    double to_left_lo, to_left_hi;
    double to_right_lo, to_right_hi;
    double to_forward_lo, to_forward_hi;

    std::string transition(double angle) {
      if (to_left_lo < angle && angle < to_left_hi) {
        return "left";
      } else if (to_right_lo < angle && angle < to_right_hi) {
        return "right";
      } else if (to_forward_lo < angle && angle < to_forward_hi) {
        return "forward";
      } else {
        return name;
      }
    }
  };

      // // Navigate to target
      // if (-fsm_rad < target_angle && target_angle < fsm_rad) {
      //   left_speed = -wheel_speed;
      //   right_speed = -wheel_speed;
      // } else if (target_angle < -fsm_rad) {
      //   left_speed = -wheel_speed * turn_factor;
      //   right_speed = wheel_speed;
      // } else {
      //   left_speed = wheel_speed;
      //   right_speed = -wheel_speed * turn_factor;
      // }

  double rad_factor = 1.0;

  std::unordered_map<std::string, State> fsm{{
    {"forward", {"forward",
      -wheel_speed, -wheel_speed,
      -2_pi, -fsm_rad,
      fsm_rad, 2_pi,
      1, -1
    }},
    {"left",    {"left",
      -wheel_speed * turn_factor, wheel_speed,
      1, -1,
      1, -1,
      -fsm_rad/rad_factor, 2_pi
    }},
    {"right",   {"right",
      wheel_speed, -wheel_speed * turn_factor,
      1, -1,
      1, -1,
      -2_pi, fsm_rad/rad_factor
    }}
  }};

  std::string cstate("forward");


  cerr << "time,dlon,dlat,dyaw,left,right,strut,pleft,pright,pstrut,x,z\n";

  long time_ms = 0;

  while (time_ms < TIME_STOP_MS) {

    auto chassis_transform = adabot.chassis->getTransform();
    auto yaw = chassis_transform.rotation().eulerAngles(2, 0, 2).y();
    auto pos = chassis_transform.translation();

    double pleft = left_speed;
    double pright = right_speed;
    double pstrut = strut_extension;

    //
    // Update Adabot control parameters
    //
    {
      // Get angle and distance to target
      auto local_to_global = AngleAxisd(yaw, Vector3d(0, 1, 0));

      auto Va = local_to_global * Vector3d(1, 0, 0);
      auto Vb = targets[target_idx] - pos;
      auto Vn = Vector3d(0, 1, 0);

      target_angle = std::atan2(Va.cross(Vb).dot(Vn), Va.dot(Vb));
      target_dist = (pos - targets[target_idx]).norm();

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

      cstate = fsm[cstate].transition(target_angle);
      left_speed = fsm[cstate].left_speed;
      right_speed = fsm[cstate].right_speed;

      adabot.left_speed = left_speed;
      adabot.right_speed = right_speed;
      adabot.strut_extension = strut_extension;
    }


    // Step forward in time and then measure changes
    next_update_time_ms += UPDATE_STEP_MS;
    while (time_ms < next_update_time_ms) {
      adabot.step();
      time_ms += TIME_STEP_MS;

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


    //
    // Output ANN training data
    //
    {
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

      // cerr << "time,dlon,dlat,dyaw,left,right,strut,pleft,pright,pstrut,x,z\n";
      cerr << adabot.world->getTime() << ","
           << dlon << ","
           << dlat << ","
           << dyaw << ","
           << left_speed << ","
           << right_speed << ","
           << strut_extension << ","
           << pleft << ","
           << pright << ","
           << pstrut
           << "," << new_pos.x() << "," << new_pos.z()
           << "\n";

      yaw = new_yaw;
    }

  }
  adabot.log(cout);

  return 0;
}
