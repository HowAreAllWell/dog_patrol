#pragma once

#include <cstdint>
#include <optional>

namespace vision_demo_host {

// Tracks the latest authoritative mission state sequence. Equal sequences are
// current; strictly newer sequences are accepted with uint32 wraparound; older
// snapshots are rejected.
class MissionStateSequenceCursor {
 public:
  bool AcceptsCurrentOrNewer(const std::uint32_t state_seq) {
    if (!latest_state_seq_.has_value()) {
      latest_state_seq_ = state_seq;
      return true;
    }
    if (state_seq == latest_state_seq_.value()) {
      return true;
    }
    if (static_cast<std::int32_t>(state_seq - latest_state_seq_.value()) <= 0) {
      return false;
    }
    latest_state_seq_ = state_seq;
    return true;
  }

 private:
  std::optional<std::uint32_t> latest_state_seq_;
};

}  // namespace vision_demo_host
