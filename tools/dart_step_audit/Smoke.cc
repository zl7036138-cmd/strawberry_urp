#include <dart/simulation/World.hpp>
#include <dart/dynamics/Skeleton.hpp>
#include <dart/dynamics/RevoluteJoint.hpp>
#include <dart/dynamics/BodyNode.hpp>
int main() {
  auto world = dart::simulation::World::create();
  auto skeleton = dart::dynamics::Skeleton::create("audit_smoke");
  dart::dynamics::RevoluteJoint::Properties properties;
  properties.mName = "panda_joint5";
  properties.mActuatorType = dart::dynamics::Joint::SERVO;
  auto pair = skeleton->createJointAndBodyNodePair<dart::dynamics::RevoluteJoint>(nullptr, properties);
  world->setGravity(Eigen::Vector3d::Zero());
  world->setTimeStep(0.001);
  world->addSkeleton(skeleton);
  for (unsigned i = 0; i < 20; ++i) {
    pair.first->setCommand(0, 0.1);
    world->step();
  }
  return 0;
}
