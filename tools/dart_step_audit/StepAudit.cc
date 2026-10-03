// Development-only ABI-specific interposer. Observes World::step boundaries;
// invokes the original exactly once with the original resetCommand argument.
// Never writes joint state, commands, actuator modes, or collision settings.
#include <dart/simulation/World.hpp>
#include <dart/dynamics/Skeleton.hpp>
#include <dart/dynamics/Joint.hpp>
#include <dlfcn.h>
#include <cstdlib>
#include <cstdio>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <stdexcept>

namespace {
using Original = void (*)(dart::simulation::World *, bool);
Original original() {
  static auto function = reinterpret_cast<Original>(
      dlsym(RTLD_NEXT, "_ZN4dart10simulation5World4stepEb"));
  if (!function) {
    std::fputs("DART audit: original World::step unavailable\n", stderr);
    std::abort();
  }
  return function;
}
struct Output {
  std::ofstream stream;
  std::mutex mutex;
  bool enabled = false;
  unsigned calls = 0;
  Output() {
    const char *path = std::getenv("STRAWBERRY_DART_AUDIT_OUTPUT");
    if (!path || !*path) return;
    // Exclusive claim: never truncate an old diagnostic. fopen's x mode is
    // supported by glibc; the file then belongs to this process.
    FILE *claim = std::fopen(path, "wx");
    if (!claim) throw std::runtime_error("DART audit output already exists or cannot be created");
    std::fclose(claim);
    stream.open(path, std::ios::out | std::ios::app);
    if (!stream) throw std::runtime_error("DART audit output cannot be opened");
    stream << std::setprecision(17);
    enabled = true;
  }
};
void scalar(std::ostream &stream, double value) {
  if (std::isfinite(value)) stream << value; else stream << "null";
}
void record(const char *phase, dart::simulation::World *world, bool resetCommand) {
  static Output output;
  if (!output.enabled) return;
  std::lock_guard<std::mutex> guard(output.mutex);
  for (std::size_t i = 0; i < world->getNumSkeletons(); ++i) {
    auto skeleton = world->getSkeleton(i);
    auto joint = skeleton->getJoint("panda_joint5");
    if (!joint || joint->getNumDofs() != 1) continue;
    auto &s = output.stream;
    s << "{\"schema_version\":1,\"phase\":\"" << phase << "\",\"dart_time_sec\":";
    scalar(s, world->getTime());
    s << ",\"reset_command\":" << (resetCommand ? "true" : "false")
      << ",\"joint_pointer\":" << reinterpret_cast<std::uintptr_t>(joint)
      << ",\"skeleton_pointer\":" << reinterpret_cast<std::uintptr_t>(skeleton.get())
      << ",\"skeleton_dof_count\":" << skeleton->getNumDofs()
      << ",\"dof_index\":" << joint->getIndexInSkeleton(0)
      << ",\"actuator_type\":" << static_cast<int>(joint->getActuatorType())
      << ",\"position_rad\":";
    scalar(s, joint->getPosition(0));
    s << ",\"velocity_rad_s\":"; scalar(s, joint->getVelocity(0));
    s << ",\"command\":"; scalar(s, joint->getCommand(0));
    s << ",\"force\":"; scalar(s, joint->getForce(0));
    s << ",\"constraint_impulse\":"; scalar(s, joint->getConstraintImpulse(0));
    s << ",\"velocity_change\":"; scalar(s, joint->getVelocityChange(0));
    s << "}\n";
  }
  if (++output.calls % 100 == 0) output.stream.flush();
}
}

void dart::simulation::World::step(bool resetCommand) {
  auto call = original();
  record("BEFORE_DART_STEP", this, resetCommand);
  call(this, resetCommand);
  record("AFTER_DART_STEP", this, resetCommand);
}
