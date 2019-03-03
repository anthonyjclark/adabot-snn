
#include <iostream>
using std::cerr;
using std::cout;
using std::endl;

#include <sstream>
using std::istringstream;

#include "../ugv.hpp"

int
main(int argc, char const* argv[])
{
  constexpr double TIME_STEP = 0.005;
  constexpr double TIME_STOP = 1.0;
  constexpr double PRINT_STEP = 0.01;

  if (argc < 2) {
    cerr << "Not enough program arguments." << endl;
    return 1;
  }

  istringstream iss(argv[1]);

  double wheel_base = 12_cm;
  double track_width = 20_cm;
  double chassis_height = 3_cm;
  double wheel_radius = 8_cm;
  double wheel_thickness = 1.5_cm;
  double weg_count = 5;
  double weg_radius = 0.25_cm;

  UGV adabot(wheel_base,
             track_width,
             chassis_height,
             wheel_radius,
             wheel_thickness,
             weg_count,
             weg_radius);

  adabot.world->setTimeStep(TIME_STEP);

  std::cout << "time,x,y,z\n";

  double print_time = 0;

  while (adabot.world->getTime() < TIME_STOP + TIME_STEP / 10.0) {
    adabot.world->step();

    auto chassis_transform = adabot.ugv->getBodyNode("chassis")->getTransform();
    auto chassis_translation = chassis_transform.translation();

    if (adabot.world->getTime() >= print_time) {
      std::cout << adabot.world->getTime() << "," << chassis_translation.x()
                << "," << chassis_translation.y() << ","
                << chassis_translation.z() << "\n";

      print_time += PRINT_STEP;
    }
  }

  return 0;
}
