// SPDX-License-Identifier: Apache-2.0
#pragma once
namespace strawberry {
enum class Operation { None, Attach, Detach };
class AttachmentPolicy {
 public:
  void Request(bool attach) { desired = attach; }
  bool Attached() const { return attached; }
  Operation Next(bool childAvailable, bool otherSupport) const {
    if (!desired && attached) return Operation::Detach;
    if (desired && !attached && childAvailable && !otherSupport) return Operation::Attach;
    return Operation::None;
  }
  void Acknowledge(Operation operation) {
    if (operation == Operation::Attach) attached = true;
    if (operation == Operation::Detach) attached = false;
  }
 private:
  bool desired = false;
  bool attached = false;
};
}
