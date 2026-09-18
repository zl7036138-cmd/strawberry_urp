// SPDX-License-Identifier: Apache-2.0
// Uses the documented gz-sim8 DetachableJoint component protocol, but never
// attaches at insertion and never adds a second incoming fruit constraint.
#include <atomic>
#include <stdexcept>
#include <string>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/empty.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/components/DetachableJoint.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <sdf/Element.hh>
#include "AttachmentPolicy.hh"

namespace strawberry {
class LazyDetachableJoint final : public gz::sim::System,
    public gz::sim::ISystemConfigure, public gz::sim::ISystemPreUpdate {
 public:
  void Configure(const gz::sim::Entity &entity,
      const std::shared_ptr<const sdf::Element> &sdf,
      gz::sim::EntityComponentManager &ecm, gz::sim::EventManager &) override {
    gz::sim::Model model(entity);
    if (!model.Valid(ecm)) throw std::runtime_error("Lazy attachment needs a model");
    parent = model.LinkByName(ecm, sdf->Get<std::string>("parent_link"));
    if (parent == gz::sim::kNullEntity) throw std::runtime_error("Attachment parent missing");
    childModel = sdf->Get<std::string>("child_model");
    childLink = sdf->Get<std::string>("child_link");
    publisher = transport.Advertise<gz::msgs::StringMsg>(sdf->Get<std::string>("output_topic"));
    if (!publisher ||
        !transport.Subscribe(sdf->Get<std::string>("attach_topic"), &LazyDetachableJoint::Attach, this) ||
        !transport.Subscribe(sdf->Get<std::string>("detach_topic"), &LazyDetachableJoint::Detach, this))
      throw std::runtime_error("Attachment transport configuration failed");
  }
  void PreUpdate(const gz::sim::UpdateInfo &, gz::sim::EntityComponentManager &ecm) override {
    const auto intent = requested.exchange(Operation::None);
    if (intent != Operation::None) policy.Request(intent == Operation::Attach);
    gz::sim::Entity modelId = gz::sim::kNullEntity;
    unsigned matches = 0;
    ecm.Each<gz::sim::components::Model, gz::sim::components::Name>(
      [&](const auto &id, const auto *, const auto *name) {
        if (name->Data() == childModel) { modelId = id; ++matches; }
        return true;
      });
    const auto child = matches == 1 ? ecm.EntityByComponents(
        gz::sim::components::Link(), gz::sim::components::ParentEntity(modelId),
        gz::sim::components::Name(childLink)) : gz::sim::kNullEntity;
    bool otherSupport = false;
    ecm.Each<gz::sim::components::DetachableJoint>(
      [&](const auto &id, const auto *joint) {
        if (id != jointId && joint->Data().childLink == child) otherSupport = true;
        return true;
      });
    const auto operation = policy.Next(child != gz::sim::kNullEntity, otherSupport);
    if (operation == Operation::Detach) {
      ecm.RequestRemoveEntity(jointId);
      jointId = gz::sim::kNullEntity;
      policy.Acknowledge(operation);
    } else if (operation == Operation::Attach) {
      jointId = ecm.CreateEntity();
      ecm.CreateComponent(jointId, gz::sim::components::DetachableJoint({parent, child, "fixed"}));
      policy.Acknowledge(operation);
    }
    // Repeat state on idempotent requests: the ROS bridge may miss insertion.
    // Missing/ambiguous children must not confirm readiness.
    if (child != gz::sim::kNullEntity &&
        (intent != Operation::None || operation != Operation::None || !initialPublished)) {
      gz::msgs::StringMsg state;
      state.set_data(policy.Attached() ? "attached" : "detached");
      publisher.Publish(state);
      initialPublished = true;
    }
  }
 private:
  void Attach(const gz::msgs::Empty &) { requested.store(Operation::Attach); }
  void Detach(const gz::msgs::Empty &) { requested.store(Operation::Detach); }
  AttachmentPolicy policy;
  gz::sim::Entity parent = gz::sim::kNullEntity;
  gz::sim::Entity jointId = gz::sim::kNullEntity;
  std::string childModel, childLink;
  bool initialPublished = false;
  std::atomic<Operation> requested{Operation::None};
  // Destroy transport before callback-accessed atomic state.
  gz::transport::Node transport;
  gz::transport::Node::Publisher publisher;
};
}
GZ_ADD_PLUGIN(strawberry::LazyDetachableJoint, gz::sim::System,
    strawberry::LazyDetachableJoint::ISystemConfigure,
    strawberry::LazyDetachableJoint::ISystemPreUpdate)
