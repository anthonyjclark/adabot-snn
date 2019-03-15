
#ifndef UGV_HPP
#define UGV_HPP

#ifdef VISUALIZE
#include "../logger-cpp/src/logger.hpp"
#endif

#include <string>
using std::string;

#include <tuple>
using std::get;
using std::make_tuple;
using std::tie;
using std::tuple;

#include <vector>
using std::vector;

#include <dart/dart.hpp>
using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::math::suffixes;
using namespace Eigen;

#include "extras/pd_controller.hpp"
#include "extras/utilities.hpp"

class UGV
{
public:
  // General parameters
  const double MATERIAL_DENSITY = 700_kg_per_m3;
  const double MATERIAL_RESTITUTION = 0;//0.05;
  // const double MATERIAL_FRICTION = 0.8;
  const double MATERIAL_FRICTION;
  const double GRAVITY = -9.80665;

  // 20 rad/s
  // 191 RPM
  // 0.6 m/s (3 cm radius wheel)
  // 1.34 mph (3 cm radius wheel)
  // average walking speed is 1.4 m/s (3.1 mph)
  const double MAX_ABS_RADS = 20;

#ifdef VISUALIZE
  // Visualization parameters
  const double VIS_STEP = 1.0 / 10.0;
  const double VIS_SCALE = 10.0;
  double next_vis_update_time = 0.0;
  bool vis_updated = false;
  review::logger logger{ VIS_STEP };
#endif

  WorldPtr world;
  SkeletonPtr ground;
  SkeletonPtr ugv;
  BodyNode* chassis;

  vector<Joint*> wheel_joints;
  vector<Joint*> strut_joints;

  double wheel_base;
  double track_width;
  double chassis_height;
  double wheel_radius;
  double wheel_thickness;
  double strut_count;
  double strut_radius;

  // Control
  double left_speed = 0.0;
  double right_speed = 0.0;
  double strut_extension = 0.0;

  // The PD controller can be reused by all wegs
  PDController strut_pd;

private:
#ifdef VISUALIZE
  void add_frame_to_rl()
  {

    logger.new_frame();

    for (size_t skel_idx = 0; skel_idx < world->getNumSkeletons(); ++skel_idx) {

      auto skel = world->getSkeleton(skel_idx);
      for (const auto& bnode : skel->getBodyNodes()) {

        auto T = bnode->getTransform();
        auto trans = T.translation() * VIS_SCALE;
        Quaterniond quat(T.rotation());

        logger.add_to_frame(bnode->getName(),
                            trans.x(),
                            trans.y(),
                            trans.z(),
                            quat.x(),
                            quat.y(),
                            quat.z(),
                            quat.w());
      }
    }
  }
#endif

  auto create_chassis()
  {
    const string name{ "chassis" };

    // Create chassis shape
    ShapePtr shape{ new BoxShape(
      Vector3d{ wheel_base, chassis_height, track_width }) };
    const double mass = MATERIAL_DENSITY * shape->getVolume();

    // Attach chassis to the world with a free joint
    FreeJoint::Properties joint_prop;
    joint_prop.mName = name + "_joint";

    // Set chassis dynamic properties
    BodyNode::Properties body_prop;
    body_prop.mName = name;
    body_prop.mRestitutionCoeff = MATERIAL_RESTITUTION;
    body_prop.mFrictionCoeff = MATERIAL_FRICTION;
    body_prop.mInertia.setMass(mass);
    body_prop.mInertia.setMoment(shape->computeInertia(mass));

    // Create the chassis and add to ugv skeleton
    auto body_joint_pair = ugv->createJointAndBodyNodePair<FreeJoint>(
      nullptr, joint_prop, body_prop);

    // Set chassis as collidable and dynamic
    body_joint_pair.second
      ->createShapeNodeWith<CollisionAspect, DynamicsAspect>(shape);

#ifdef VISUALIZE
    logger.add_box(name,
                   wheel_base * VIS_SCALE,
                   chassis_height * VIS_SCALE,
                   track_width * VIS_SCALE,
                   { 0.2, 0.8, 0.2, 1.0 });
#endif

    return body_joint_pair;
  }

  auto create_wheel(BodyNode* parent,
                    const string& name,
                    double x_offset,
                    double z_offset)
  {
    const double wr2 = wheel_radius * 2.0;
    ShapePtr shape{ new EllipsoidShape{
      Vector3d{ wr2, wr2, wheel_thickness } } };
    const double mass = MATERIAL_DENSITY * shape->getVolume();

    // Setup the wheel properties
    string joint_name = name + "_joint";
    RevoluteJoint::Properties joint_prop;
    joint_prop.mName = joint_name;

    Isometry3d tf(Isometry3d::Identity());
    tf.translation() = Vector3d(x_offset, 0, z_offset);
    joint_prop.mT_ParentBodyToJoint = tf;

    BodyNode::Properties body_prop;
    body_prop.mName = name;
    body_prop.mRestitutionCoeff = MATERIAL_RESTITUTION;
    body_prop.mFrictionCoeff = MATERIAL_FRICTION;
    body_prop.mInertia.setMass(mass);
    body_prop.mInertia.setMoment(shape->computeInertia(mass));

    // Create the joint-node pair
    auto body_joint_pair = ugv->createJointAndBodyNodePair<RevoluteJoint>(
      parent, joint_prop, body_prop);

    // Set the shape of the body
    body_joint_pair.second
      ->createShapeNodeWith<CollisionAspect, DynamicsAspect>(shape);

    // Set the actuator type of the wheel joint
    body_joint_pair.first->setActuatorType(Joint::VELOCITY);

    wheel_joints.push_back(ugv->getJoint(joint_name));

#ifdef VISUALIZE
    logger.add_ellipsoid(name,
                         wr2 * VIS_SCALE,
                         wr2 * VIS_SCALE,
                         wheel_thickness * VIS_SCALE,
                         { 0.299, 0.587, 0.114, 1.0 });
#endif

    //
    // struts
    //
    ShapePtr strut_shape{ new SphereShape{ strut_radius } };
    const double strut_mass = MATERIAL_DENSITY * strut_shape->getVolume();
    PrismaticJoint::Properties strut_joint_prop;
    BodyNode::Properties strut_body_prop;
    strut_body_prop.mRestitutionCoeff = MATERIAL_RESTITUTION;
    strut_body_prop.mFrictionCoeff = MATERIAL_FRICTION;
    strut_body_prop.mInertia.setMass(strut_mass);
    strut_body_prop.mInertia.setMoment(strut_shape->computeInertia(strut_mass));

    for (size_t strut_idx = 0; strut_idx < strut_count; ++strut_idx) {

      string strut_name = name + "_strut" + std::to_string(strut_idx);
      strut_body_prop.mName = strut_name;

      string strut_joint_name =
        name + "_strut_joint" + std::to_string(strut_idx);
      strut_joint_prop.mName = strut_joint_name;

      // Move the strut into place
      Isometry3d tf_strut{ Isometry3d::Identity() };
      auto vector_to_point = Vector3d(0, wheel_radius - strut_radius, 0);
      auto vector_transform =
        AngleAxisd(2_pi * strut_idx / strut_count, Vector3d(0, 0, 1));
      tf_strut.translation() = vector_transform * vector_to_point;

      strut_joint_prop.mT_ParentBodyToJoint = tf_strut;

      auto strut_pair = ugv->createJointAndBodyNodePair<PrismaticJoint>(
        body_joint_pair.second, strut_joint_prop, strut_body_prop);
      strut_pair.second->createShapeNodeWith<CollisionAspect, DynamicsAspect>(
        strut_shape);
      strut_pair.first->setActuatorType(Joint::VELOCITY);

      strut_pair.first->setAxis(vector_transform * vector_to_point);
      strut_joints.push_back(ugv->getJoint(strut_joint_name));

#ifdef VISUALIZE
      logger.add_sphere(strut_name, strut_radius * VIS_SCALE * 2);
#endif
    }

    return body_joint_pair;
  }

  void create_ground()
  {
    auto ground_depth = 1.0;

    ground = Skeleton::create("ground");
    auto ground_body = ground->createJointAndBodyNodePair<WeldJoint>().second;
    // ground_body->setRestitutionCoeff(MATERIAL_RESTITUTION);
    ground_body->setName("ground");
    ShapePtr ground_box(new BoxShape(Vector3d(100, ground_depth, 100)));
    ground_body->createShapeNodeWith<CollisionAspect, DynamicsAspect>(
      ground_box);

    // Shift the ground so that its top is at y=0
    Isometry3d ground_tf(Isometry3d::Identity());
    ground_tf.translate(Vector3d(0.0, -ground_depth / 2.0, 0.0));
    ground_body->getParentJoint()->setTransformFromParentBodyNode(ground_tf);
  }

public:
  UGV(double wheel_base,
      double track_width,
      double chassis_height,
      double wheel_radius,
      double wheel_thickness,
      double strut_count,
      double strut_radius,
      double time_step,
      double friction)
    : MATERIAL_FRICTION(friction)
    , world(new World)
    , wheel_base(wheel_base)
    , track_width(track_width)
    , chassis_height(chassis_height)
    , wheel_radius(wheel_radius)
    , wheel_thickness(wheel_thickness)
    , strut_count(strut_count)
    , strut_radius(strut_radius)
    , strut_pd(0.5, 0.05, time_step)
  {

    world->setTimeStep(time_step);

    ugv = Skeleton::create("ugv");

    // Create the chassis as part of the UGV and attach it to the world
    FreeJoint* chassis_joint;
    tie(chassis_joint, chassis) = create_chassis();

    // Create the wheels and wheel struts
    vector<tuple<string, double, double>> wheel_info{
      make_tuple("front-right-wheel", wheel_base / 2.0, track_width / 2.0),
      make_tuple("front-left-wheel", wheel_base / 2.0, -track_width / 2.0),
      make_tuple("back-right-wheel", -wheel_base / 2.0, track_width / 2.0),
      make_tuple("back-left-wheel", -wheel_base / 2.0, -track_width / 2.0)
    };

    for (const auto& wi : wheel_info) {
      create_wheel(chassis, get<0>(wi), get<1>(wi), get<2>(wi));
    }

    // Create ground
    create_ground();

    // Create the world and add the ugv skeleton
    world->addSkeleton(ugv);
    world->addSkeleton(ground);
    world->setGravity(Vector3d(0.0, GRAVITY, 0.0));

    // Position the UGV above ground
    Vector6d ugv_positions(Vector6d::Zero());
    ugv_positions[4] = wheel_radius + 1_cm;
    chassis_joint->setPositions(ugv_positions);
  }

  void step()
  {
    world->step();

    wheel_joints.at(0)->setCommand(0, right_speed);
    wheel_joints.at(1)->setCommand(0, left_speed);
    wheel_joints.at(2)->setCommand(0, right_speed);
    wheel_joints.at(3)->setCommand(0, left_speed);

    for (const auto& strut : strut_joints) {

      // Get PD output
      auto y = strut->getPosition(0);
      auto dy = strut->getVelocity(0);
      auto u = strut_pd.get_output(strut_extension, y, dy);

      // Struts only have 1 DOF, so index is 0
      strut->setCommand(0, u);
    }

#ifdef VISUALIZE
    if (world->getTime() > next_vis_update_time) {
      next_vis_update_time += VIS_STEP;
      add_frame_to_rl();
      vis_updated = true;
    }
#endif
  }

  void log(std::ostream &out)
  {
#ifdef VISUALIZE
    out << logger.to_string();
  #else
    out << "Not logging visualization data.\n";
#endif
  }
};

#endif
