#include <cstdio>
#include <limits>

#include "dog_patrol_reid_provider/reid_readiness.hpp"

int main() {
  using dog_patrol_reid_provider::ReidReadiness;
  int failures = 0;
  const auto check = [&failures](bool condition, const char* label) {
    if (!condition) { ++failures; std::fprintf(stderr, "FAIL: %s\n", label); }
  };
  ReidReadiness gate(0.75);
  check(gate.startup_sequence(1.0) == 0, "no mission cannot publish readiness");
  gate.observe(10, 0, 0, 0, 1.0);
  check(gate.startup_sequence(1.1) == 10, "valid STARTUP");
  gate.observe(9, 7, 0, 0, 1.2);
  check(gate.startup_sequence(1.3) == 10, "old state ignored");
  check(gate.startup_sequence(1.8) == 0, "old state cannot refresh heartbeat");
  gate.observe(10, 0, 0, 0, 1.9);
  check(gate.startup_sequence(2.0) == 10, "current heartbeat recovers");
  gate.observe(11, 7, 0, 0, 2.1);
  check(gate.startup_sequence(2.2) == 0, "READY stops readiness");
  gate.observe(10, 0, 0, 0, 2.3);
  check(gate.startup_sequence(2.4) == 0, "late STARTUP cannot reactivate");
  gate.observe(11, 0, 0, 0, 2.5);
  gate.observe(11, 7, 0, 0, 2.6);
  check(gate.startup_sequence(2.7) == 0, "same sequence conflict locked");
  gate.observe(12, 0, 0, 0, 2.8);
  check(gate.startup_sequence(2.9) == 12, "reset uses new sequence");
  gate.observe(12, 0, 1, 0, 3.0);
  gate.observe(12, 0, 0, 0, 3.1);
  check(gate.startup_sequence(3.2) == 0, "target conflict locked");
  gate.observe(13, 0, 0, 1, 3.3);
  check(gate.startup_sequence(3.4) == 0, "STARTUP handled target invalid");
  gate.observe(14, 0, 0, 0, 3.5);
  gate.observe(0, 0, 0, 0, 3.6);
  check(gate.startup_sequence(3.7) == 0, "zero sequence invalidates readiness");
  gate.observe(14, 0, 0, 0, 3.8);
  check(gate.startup_sequence(3.9) == 14, "valid heartbeat recovers from zero");
  gate.observe(15, 255, 0, 0, 4.0);
  check(gate.startup_sequence(4.1) == 0, "unknown state cannot be ready");
  ReidReadiness wrap(0.75);
  wrap.observe(std::numeric_limits<unsigned>::max(), 0, 0, 0, 5.0);
  wrap.observe(1, 0, 0, 0, 5.1);
  check(wrap.startup_sequence(5.2) == 1, "uint32 wrap accepts new STARTUP");
  wrap.observe(std::numeric_limits<unsigned>::max(), 0, 0, 0, 5.3);
  check(wrap.startup_sequence(5.4) == 1, "pre-wrap late state rejected");
  std::printf("ReID readiness: 16 checks, %d failures\n", failures);
  return failures ? 1 : 0;
}
