// Optional read-only ECM observer. Never writes components or control commands.
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <stdexcept>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointVelocityReset.hh>
#include <sdf/Element.hh>

namespace strawberry {
class CommandAudit final : public gz::sim::System,
    public gz::sim::ISystemConfigure, public gz::sim::ISystemConfigurePriority,
    public gz::sim::ISystemUpdate,
    public gz::sim::ISystemPostUpdate {
 public:
  // All PreUpdate command writers finish before Update. Run immediately
  // BEFORE Physics::Update (default priority 0), which consumes and clears
  // JointVelocityCmd; otherwise every recorded command misleadingly is zero.
  gz::sim::System::PriorityType ConfigurePriority() override { return -1; }
  void Configure(const gz::sim::Entity &, const std::shared_ptr<const sdf::Element> &sdf,
      gz::sim::EntityComponentManager &, gz::sim::EventManager &) override {
    stream.open(sdf->Get<std::string>("output_file"), std::ios::out | std::ios::app);
    if (!stream) throw std::runtime_error("Cannot open command-audit output");
    stream << std::setprecision(17);
  }
  void Update(const gz::sim::UpdateInfo &info,
      gz::sim::EntityComponentManager &ecm) override { Record("UPDATE", info, ecm); }
  void PostUpdate(const gz::sim::UpdateInfo &info,
      const gz::sim::EntityComponentManager &ecm) override { Record("POST_UPDATE", info, ecm); }
 private:
  std::ofstream stream;
  template<class Component>
  void Scalar(const gz::sim::EntityComponentManager &ecm, gz::sim::Entity id) {
    auto c = ecm.Component<Component>(id);
    if (c && !c->Data().empty() && std::isfinite(c->Data()[0])) stream << c->Data()[0];
    else stream << "null";
  }
  void Record(const char *phase, const gz::sim::UpdateInfo &info,
      const gz::sim::EntityComponentManager &ecm) {
    if (info.paused) return;
    ecm.Each<gz::sim::components::Joint, gz::sim::components::Name>(
      [&](const gz::sim::Entity &id, const auto *, const auto *name) {
        if (name->Data() != "panda_joint5") return true;
        stream << "{\"schema_version\":1,\"phase\":\"" << phase
               << "\",\"iteration\":" << info.iterations
               << ",\"sim_time_sec\":" << std::chrono::duration<double>(info.simTime).count()
               << ",\"entity\":" << id << ",\"position_rad\":";
        Scalar<gz::sim::components::JointPosition>(ecm, id);
        stream << ",\"velocity_rad_s\":";
        Scalar<gz::sim::components::JointVelocity>(ecm, id);
        stream << ",\"velocity_command_rad_s\":";
        Scalar<gz::sim::components::JointVelocityCmd>(ecm, id);
        stream << ",\"force_command_present\":"
               << (ecm.Component<gz::sim::components::JointForceCmd>(id) ? "true" : "false")
               << ",\"velocity_reset_present\":"
               << (ecm.Component<gz::sim::components::JointVelocityReset>(id) ? "true" : "false")
               << "}\n";
        return true;
      });
    if (info.iterations % 100 == 0) stream.flush();
  }
};
}
GZ_ADD_PLUGIN(strawberry::CommandAudit, gz::sim::System,
              strawberry::CommandAudit::ISystemConfigure,
              strawberry::CommandAudit::ISystemConfigurePriority,
              strawberry::CommandAudit::ISystemUpdate,
              strawberry::CommandAudit::ISystemPostUpdate)
