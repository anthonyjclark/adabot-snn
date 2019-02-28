
#ifndef UGV_HPP
#define UGV_HPP

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

#include "extras/utilities.hpp"

class UGV
{
public:
  // General parameters
  const double material_density = 700_kg_per_m3;
  const double material_restitution = 0.75;
  const double gravity = -9.80665;

  WorldPtr world;
  SkeletonPtr ground;
  SkeletonPtr ugv;

  double wheel_base;
  double track_width;
  double chassis_height; // 3cm
  double wheel_radius;
  double wheel_thickness; // 1.5cm
  double weg_count;
  double weg_radius; // 0.25cm

private:
  auto create_chassis()
  {
    const string name{ "chassis" };

    ShapePtr shape{ new BoxShape(
      Vector3d{ wheel_base, chassis_height, track_width }) };
    const double mass = material_density * shape->getVolume();

    // Setup the joint properties
    FreeJoint::Properties joint_prop;
    joint_prop.mName = name + "_joint";

    // Setup the body properties
    BodyNode::Properties body_prop;
    body_prop.mName = name;
    body_prop.mRestitutionCoeff = material_restitution;
    body_prop.mInertia.setMass(mass);
    body_prop.mInertia.setMoment(shape->computeInertia(mass));

    // Create the joint-node pair
    auto body_joint_pair = ugv->createJointAndBodyNodePair<FreeJoint>(
      nullptr, joint_prop, body_prop);

    // Set the shape of the body
    body_joint_pair.second
      ->createShapeNodeWith<CollisionAspect, DynamicsAspect>(shape);

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
    const double mass = material_density * shape->getVolume();

    // Setup the wheel properties
    RevoluteJoint::Properties joint_prop;
    joint_prop.mName = name + "_joint";

    Isometry3d tf(Isometry3d::Identity());
    tf.translation() = Vector3d(x_offset, 0, z_offset);
    joint_prop.mT_ParentBodyToJoint = tf;

    BodyNode::Properties body_prop;
    body_prop.mName = name;
    body_prop.mRestitutionCoeff = material_restitution;
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

    //
    // Wegs
    //
    ShapePtr weg_shape{ new SphereShape{ weg_radius } };
    const double weg_mass = material_density * weg_shape->getVolume();
    PrismaticJoint::Properties weg_joint_prop;
    BodyNode::Properties weg_body_prop;
    weg_body_prop.mRestitutionCoeff = material_restitution;
    weg_body_prop.mInertia.setMass(weg_mass);
    weg_body_prop.mInertia.setMoment(weg_shape->computeInertia(weg_mass));

    for (size_t weg_idx = 0; weg_idx < weg_count; ++weg_idx) {

      weg_body_prop.mName = name + "_weg" + std::to_string(weg_idx);
      weg_joint_prop.mName = name + "_weg_joint" + std::to_string(weg_idx);

      // Move the weg into place
      Isometry3d tf_weg{ Isometry3d::Identity() };
      auto vector_to_point = Vector3d(0, wheel_radius / 2.0 - weg_radius, 0);
      auto vector_transform =
        AngleAxisd(2_pi * weg_idx / weg_count, Vector3d(0, 0, 1));
      tf_weg.translation() = vector_transform * vector_to_point;

      weg_joint_prop.mT_ParentBodyToJoint = tf_weg;

      auto weg_pair = ugv->createJointAndBodyNodePair<PrismaticJoint>(
        body_joint_pair.second, weg_joint_prop, weg_body_prop);
      weg_pair.second->createShapeNodeWith<CollisionAspect, DynamicsAspect>(
        weg_shape);
      weg_pair.first->setActuatorType(Joint::VELOCITY);

      weg_pair.first->setAxis(vector_transform * vector_to_point);
    }

    return body_joint_pair;
  }

  void create_ground()
  {
    auto ground_depth = 1.0;

    ground = Skeleton::create("ground");
    auto ground_body = ground->createJointAndBodyNodePair<WeldJoint>().second;
    ground_body->setRestitutionCoeff(material_restitution);
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
      double weg_count,
      double weg_radius)
    : world(new World)
    , wheel_base(wheel_base)
    , track_width(track_width)
    , chassis_height(chassis_height)
    , wheel_radius(wheel_radius)
    , wheel_thickness(wheel_thickness)
    , weg_count(weg_count)
    , weg_radius(weg_radius)
  {
    ugv = Skeleton::create("ugv");

    // Create the chassis as part of the UGV and attach it to the world
    FreeJoint* chassis_joint;
    BodyNode* chassis_body;
    tie(chassis_joint, chassis_body) = create_chassis();

    // Create the wheels and wheel struts
    vector<tuple<string, double, double>> wheel_info{
      make_tuple("front-right-wheel", wheel_base / 2.0, track_width / 2.0),
      make_tuple("front-left-wheel", wheel_base / 2.0, -track_width / 2.0),
      make_tuple("back-right-wheel", -wheel_base / 2.0, track_width / 2.0),
      make_tuple("back-left-wheel", -wheel_base / 2.0, -track_width / 2.0)
    };

    for (const auto& wi : wheel_info) {
      create_wheel(chassis_body, get<0>(wi), get<1>(wi), get<2>(wi));
    }

    // Create ground
    create_ground();

    // Create the world and add the ugv skeleton
    world->addSkeleton(ugv);
    world->addSkeleton(ground);
    world->setGravity(Vector3d(0.0, gravity, 0.0));

    // Position the UGV above ground
    Vector6d ugv_positions(Vector6d::Zero());
    ugv_positions[4] = wheel_radius + 1_cm;
    chassis_joint->setPositions(ugv_positions);
  }
};

#endif
