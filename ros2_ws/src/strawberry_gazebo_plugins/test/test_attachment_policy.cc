#include <cstdlib>
#include "AttachmentPolicy.hh"
using strawberry::AttachmentPolicy;
using strawberry::Operation;
#define CHECK(x) do { if (!(x)) return EXIT_FAILURE; } while (0)
int main() {
  AttachmentPolicy p;
  CHECK(!p.Attached());
  CHECK(p.Next(true, false) == Operation::None);
  p.Request(true);
  CHECK(p.Next(false, false) == Operation::None);
  CHECK(p.Next(true, true) == Operation::None);
  CHECK(p.Next(true, false) == Operation::Attach);
  p.Request(false);
  CHECK(p.Next(true, false) == Operation::None);
  p.Request(true);
  p.Acknowledge(p.Next(true, false));
  CHECK(p.Attached());
  CHECK(p.Next(true, false) == Operation::None);
  p.Request(false);
  CHECK(p.Next(true, true) == Operation::Detach);
  p.Acknowledge(Operation::Detach);
  CHECK(!p.Attached());
  CHECK(p.Next(true, false) == Operation::None);
  return EXIT_SUCCESS;
}
