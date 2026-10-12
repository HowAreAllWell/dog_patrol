#pragma once

#include <cstdint>
#include <tuple>

namespace dog_patrol_reid_provider {

// Only current, conflict-free STARTUP heartbeats can authorize capability output.
// Model initialization never changes MissionState or creates a target identity.
class ReidReadiness {
 public:
  explicit ReidReadiness(double timeout) : timeout_(timeout) {}

  void observe(uint32_t seq, uint8_t state, uint64_t target,
               uint64_t handled, double now) {
    if (seq == 0) { last_heartbeat_ = -1.0; return; }
    const auto signature = std::make_tuple(state, target, handled);
    if (sequence_ != 0) {
      const uint32_t delta = seq - sequence_;
      if (delta >= (uint32_t{1} << 31)) { return; }
      if (delta == 0) {
        if (signature != signature_) { conflicted_ = true; }
        last_heartbeat_ = now;
        return;
      }
    }
    sequence_ = seq;
    signature_ = signature;
    conflicted_ = false;
    last_heartbeat_ = now;
  }

  uint32_t startup_sequence(double now) const {
    return sequence_ != 0 && !conflicted_ && last_heartbeat_ >= 0.0 &&
                   now - last_heartbeat_ <= timeout_ &&
                   signature_ == std::make_tuple(uint8_t{0}, uint64_t{0}, uint64_t{0})
               ? sequence_ : 0;
  }

 private:
  double timeout_;
  double last_heartbeat_ = -1.0;
  uint32_t sequence_ = 0;
  std::tuple<uint8_t, uint64_t, uint64_t> signature_{};
  bool conflicted_ = false;
};
}  // namespace dog_patrol_reid_provider
