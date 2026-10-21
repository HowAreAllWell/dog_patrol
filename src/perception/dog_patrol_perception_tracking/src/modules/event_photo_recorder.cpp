#include "dog_patrol_perception_tracking/modules/event_photo_recorder.hpp"

#include <algorithm>
#include <cerrno>
#include <condition_variable>
#include <cstdlib>
#include <ctime>
#include <deque>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <map>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <system_error>
#include <thread>
#include <tuple>
#include <utility>

#include <fcntl.h>
#include <unistd.h>
#include <opencv2/imgcodecs.hpp>

namespace dog_patrol_perception_tracking {
namespace {

bool Newer(std::uint32_t candidate, std::uint32_t current) {
  const auto delta = candidate - current;
  return delta != 0U && delta < 0x80000000U;
}

bool Active(const MissionSnapshot &mission) {
  return mission.target_id > 0 &&
         (mission.phase == MissionPhase::kConfirmTarget ||
          mission.phase == MissionPhase::kApproachTarget ||
          mission.phase == MissionPhase::kVerifyIdentity ||
          mission.phase == MissionPhase::kTrackIntruder);
}

std::filesystem::path PhotoRoot(const std::string &root) {
  if (root.empty()) {
    throw std::invalid_argument("photo root must not be empty");
  }
  if (root == "~" || root.rfind("~/", 0) == 0) {
    const char *home = std::getenv("HOME");
    if (home == nullptr || *home == '\0') {
      throw std::invalid_argument("HOME must be set to expand photo root");
    }
    return root == "~" ? std::filesystem::path(home)
                       : std::filesystem::path(home) / root.substr(2);
  }
  return root;
}

std::string Filename(std::uint64_t stamp_ns, int semantic_id) {
  const auto seconds = static_cast<std::time_t>(stamp_ns / 1000000000ULL);
  std::tm local{};
  if (localtime_r(&seconds, &local) == nullptr) {
    throw std::runtime_error("photo timestamp is outside local-time range");
  }
  std::ostringstream name;
  name << std::put_time(&local, "%Y%m%d_%H%M%S") << "_t" << semantic_id;
  return name.str();
}

bool ValidConfidence(std::uint8_t event,
                     const std::optional<EventPhotoRecorder::VoiceConfidence> &confidence) {
  if (!confidence) {
    return true;  // Older/injected providers have no measurement; persist null, not zero.
  }
  const auto &value = *confidence;
  constexpr std::uint8_t percentages[]{0, 50, 75, 90, 100, 100, 100};
  if (event == EventPhotoRecorder::WAIT_STARTED || value.vote_count > 6 ||
      value.vote_count != value.matching_channels.size() ||
      value.confidence_percent != percentages[value.vote_count] ||
      (event == EventPhotoRecorder::PASSED) != (value.vote_count > 0)) {
    return false;
  }
  bool seen[6]{};
  for (auto channel : value.matching_channels) {
    if (channel >= 6 || seen[channel]) {
      return false;
    }
    seen[channel] = true;
  }
  return true;
}

void WriteBytes(int fd, const unsigned char *bytes, std::size_t size) {
  std::size_t offset = 0;
  while (offset < size) {
    const auto count = ::write(fd, bytes + offset, size - offset);
    if (count < 0 && errno == EINTR) {
      continue;
    }
    if (count <= 0) {
      throw std::system_error(count == 0 ? EIO : errno, std::generic_category(), "write evidence");
    }
    offset += static_cast<std::size_t>(count);
  }
}

void CloseFile(int &fd) {
  const int result = ::close(fd);
  fd = -1;
  if (result != 0) {
    throw std::system_error(errno, std::generic_category(), "close evidence");
  }
}

void PublishJson(const std::filesystem::path &path, const std::string &json) {
  const auto temporary = std::filesystem::path(path.string() + ".tmp");
  int fd = ::open(temporary.c_str(), O_CREAT | O_EXCL | O_WRONLY | O_CLOEXEC, 0600);
  if (fd < 0) {
    throw std::system_error(errno, std::generic_category(), "create voice JSON temporary file");
  }
  try {
    WriteBytes(fd, reinterpret_cast<const unsigned char *>(json.data()), json.size());
    CloseFile(fd);
    // Publish a complete JSON without overwriting any prior record.
    if (::link(temporary.c_str(), path.c_str()) != 0) {
      throw std::system_error(errno, std::generic_category(), "publish voice JSON");
    }
  } catch (...) {
    if (fd >= 0) {
      ::close(fd);
    }
    std::error_code ignored;
    std::filesystem::remove(temporary, ignored);
    throw;
  }
  std::error_code ignored;
  std::filesystem::remove(temporary, ignored);
}

}  // namespace

class EventPhotoRecorder::Impl {
 public:
  Impl(Config config, ErrorSink error_sink)
      : config_(std::move(config)), root_(PhotoRoot(config_.root)),
        error_sink_(std::move(error_sink)) {
    if (config_.frame_history_capacity == 0 || config_.write_queue_capacity == 0 ||
        config_.max_frame_age_ns == 0) {
      throw std::invalid_argument("photo queue capacities and max frame age must be positive");
    }
    worker_ = std::thread([this] { Run(); });
  }

  ~Impl() {
    Flush();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stopping_ = true;
    }
    wake_.notify_one();
    worker_.join();
  }

  void ObserveFrame(const std::optional<MissionSnapshot> &mission, int trusted_target,
                    const std::vector<MissionEventAction> &events,
                    const SourceFrameMetadata &metadata, const cv::Mat &image,
                    std::uint64_t mission_stamp_ns) {
    std::vector<std::string> errors;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const auto stamp = metadata.source_timestamp_ns;
      if (stamp == 0 || stamp < latest_frame_stamp_) {
        return;
      }
      if (mission && latest_mission_ && mission->state_seq != latest_mission_->state_seq &&
          !Newer(mission->state_seq, latest_mission_->state_seq)) {
        return;
      }
      const auto previous_frame_stamp = latest_frame_stamp_;
      latest_frame_stamp_ = stamp;
      UpdateMission(mission, mission_stamp_ns == 0 ? stamp : mission_stamp_ns,
                    mission_stamp_ns == 0 ? previous_frame_stamp : mission_stamp_ns);

      const bool valid_image = config_.photos_enabled && !image.empty() && image.type() == CV_8UC3 &&
                               image.cols == metadata.image_width &&
                               image.rows == metadata.image_height;
      const auto closed_session = std::find_if(sessions_.rbegin(), sessions_.rend(), [&](const auto &session) {
        return session.closed_at && stamp >= session.started_at && stamp <= *session.closed_at;
      });
      std::optional<Frame> frame;
      if (valid_image && ((mission && Active(*mission)) || !events.empty() || closed_session != sessions_.rend())) {
        frame = Frame{stamp, lifecycle_, mission && Active(*mission)
                                            ? mission->target_id : trusted_target,
                      image.clone()};
      }
      if (frame && closed_session != sessions_.rend()) {
        // Source time takes precedence over the mission snapshot observed when
        // inference finishes, even if a different target is already active.
        frame->lifecycle = closed_session->lifecycle;
        frame->target = closed_session->target;
        history_.push_back(*frame);
        while (history_.size() > config_.frame_history_capacity) {
          history_.pop_front();
        }
      } else if (frame && mission && Active(*mission) && stamp >= lifecycle_started_at_) {
        history_.push_back(*frame);
        while (history_.size() > config_.frame_history_capacity) {
          history_.pop_front();
        }
        if (trusted_target == mission->target_id) {
          last_visible_ = *frame;
        }
      }
      for (const auto &event : events) {
        if (!config_.photos_enabled) {
          break;
        }
        if (!mission || event.observed_state_seq != mission->state_seq || event.target_id <= 0) {
          Invalid("photo event does not match the current mission", errors);
          continue;
        }
        const auto key = std::make_tuple(event.observed_state_seq, event.target_id, event.event);
        if (std::find(recorded_events_.begin(), recorded_events_.end(), key) != recorded_events_.end()) {
          continue;
        }
        recorded_events_.push_back(key);
        while (recorded_events_.size() > config_.frame_history_capacity) {
          recorded_events_.pop_front();
        }
        if (event.event == PerceptionMissionEvent::kTargetConfirmed) {
          if (frame && trusted_target == event.target_id) {
            auto detection = *frame;
            detection.target = event.target_id;
            if (mission->phase == MissionPhase::kPatrol) {
              pending_detection_ = std::make_pair(mission->state_seq, detection);
            }
            Enqueue(detection, "tracking/event", errors);
          } else {
            Missing("establish-event photo has no matching visible target frame", errors);
          }
        } else if (last_visible_ && last_visible_->target == event.target_id &&
                   last_visible_->lifecycle == lifecycle_) {
          Enqueue(*last_visible_, "tracking/lost", errors);
        } else {
          Missing("target-lost photo has no trusted visible frame in this lifecycle", errors);
        }
      }
      ResolvePending(errors);
    }
    Report(errors);
  }

  void RecordVoice(std::uint32_t seq, int target, std::uint32_t round,
                   std::uint8_t event, std::uint64_t stamp,
                   std::optional<VoiceConfidence> confidence) {
    std::vector<std::string> errors;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (seq == 0 || target <= 0 || round == 0 || stamp == 0 || event > NOT_PASSED ||
          !ValidConfidence(event, confidence)) {
        Invalid("invalid voice photo event", errors);
      } else {
        const VoiceEvent value{seq, target, round, event, stamp, std::move(confidence)};
        if (!Resolve(value, errors)) {
          const auto duplicate = std::find_if(pending_.begin(), pending_.end(), [&](const auto &old) {
            return old.seq == seq && old.target == target && old.round == round &&
                   old.event == event && old.stamp == stamp;
          });
          if (duplicate == pending_.end()) {
            if (pending_.size() >= config_.frame_history_capacity) {
              pending_.pop_front();
              ++metrics_.pending_dropped;
              errors.push_back("photo pending-event queue full; oldest event discarded");
            }
            pending_.push_back(value);
          }
        }
        ResolvePending(errors);
      }
    }
    Report(errors);
  }

  void Flush() {
    std::vector<std::string> errors;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      ResolvePending(errors, true);
      if (!pending_.empty()) {
        metrics_.pending_dropped += pending_.size();
        pending_.clear();
        errors.push_back("unmatched voice photo events discarded while flushing");
      }
      drained_.wait(lock, [this] { return queue_.empty() && !writing_; });
    }
    Report(errors);
  }

  Metrics GetMetrics() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return metrics_;
  }

 private:
  struct Frame {
    std::uint64_t stamp;
    std::uint64_t lifecycle;
    int target;
    cv::Mat image;
  };
  struct Round {
    std::uint64_t started_at;
    std::optional<Frame> start_frame;
  };
  struct Session {
    std::uint32_t seq;
    int target;
    std::uint64_t lifecycle;
    std::uint64_t started_at;
    std::optional<std::uint64_t> closed_at;
    std::map<std::uint32_t, Round> rounds;
    std::uint32_t completed_through{0};
  };
  struct VoiceEvent {
    std::uint32_t seq;
    int target;
    std::uint32_t round;
    std::uint8_t event;
    std::uint64_t stamp;
    std::optional<VoiceConfidence> confidence;
  };
  struct VoiceRecord {
    VoiceEvent event;
    std::uint64_t wait_started_stamp;
    std::optional<std::uint64_t> image_stamp;
    std::string photo_status;
  };
  struct WriteJob {
    Frame frame;
    std::string directory;
    std::string filename_suffix;
    std::optional<VoiceRecord> voice;
  };

  void UpdateMission(const std::optional<MissionSnapshot> &mission, std::uint64_t stamp,
                     std::uint64_t started_at) {
    current_mission_available_ = mission.has_value();
    for (auto &session : sessions_) {
      if (!session.closed_at && (!mission || mission->phase != MissionPhase::kVerifyIdentity ||
                                mission->state_seq != session.seq || mission->target_id != session.target)) {
        session.closed_at = stamp;
      }
    }
    if (!mission || !Active(*mission)) {
      active_target_ = 0;
      last_visible_.reset();
      if (!mission || mission->phase != MissionPhase::kPatrol ||
          (pending_detection_ && pending_detection_->first != mission->state_seq)) {
        pending_detection_.reset();
      }
    } else {
      const bool restarted = latest_mission_ && Active(*latest_mission_) &&
                             mission->state_seq != latest_mission_->state_seq &&
                             static_cast<int>(mission->phase) <= static_cast<int>(latest_mission_->phase);
      if (active_target_ != mission->target_id || restarted) {
        ++lifecycle_;
        lifecycle_started_at_ = started_at;
        active_target_ = mission->target_id;
        last_visible_.reset();
        if (pending_detection_ && latest_mission_ &&
            latest_mission_->phase == MissionPhase::kPatrol &&
            pending_detection_->first == latest_mission_->state_seq &&
            pending_detection_->second.target == mission->target_id) {
          auto detection = pending_detection_->second;
          detection.lifecycle = lifecycle_;
          lifecycle_started_at_ = std::min(lifecycle_started_at_, detection.stamp);
          last_visible_ = detection;
          history_.push_back(std::move(detection));
          while (history_.size() > config_.frame_history_capacity) {
            history_.pop_front();
          }
        }
      }
      pending_detection_.reset();
      if (mission->phase == MissionPhase::kVerifyIdentity &&
          std::none_of(sessions_.begin(), sessions_.end(), [&](const auto &session) {
            return session.seq == mission->state_seq && session.target == mission->target_id &&
                   !session.closed_at;
          })) {
        // Without a state timestamp (standalone/test callers), the preceding
        // source frame is a conservative lower bound for a newly observed stage.
        for (const auto &session : sessions_) {
          if (session.seq == mission->state_seq && session.target == mission->target_id && session.closed_at) {
            // Heartbeat recovery can restart round 1 without changing state_seq.
            // Separate its images and timestamps from the previous generation.
            started_at = std::max(started_at, *session.closed_at + 1U);
          }
        }
        sessions_.push_back({mission->state_seq, mission->target_id, lifecycle_,
                             started_at, std::nullopt, {}, 0});
        while (sessions_.size() > config_.frame_history_capacity) {
          sessions_.pop_front();
        }
      }
    }
    if (mission) {
      latest_mission_ = mission;
    }
  }

  std::optional<Frame> FindFrame(const Session &session, std::uint64_t stamp) const {
    for (auto it = history_.rbegin(); it != history_.rend(); ++it) {
      if (it->lifecycle == session.lifecycle && it->target == session.target &&
          it->stamp <= stamp && stamp - it->stamp <= config_.max_frame_age_ns) {
        return *it;
      }
    }
    return std::nullopt;
  }

  // Returns false only when a later state/frame/start event may resolve this.
  bool Resolve(const VoiceEvent &event, std::vector<std::string> &errors, bool finalizing = false) {
    auto session = std::find_if(sessions_.rbegin(), sessions_.rend(), [&](const auto &value) {
      return value.seq == event.seq && value.target == event.target &&
             event.stamp >= value.started_at && (!value.closed_at || event.stamp <= *value.closed_at);
    });
    if (session == sessions_.rend()) {
      if (!current_mission_available_ && latest_mission_ &&
          latest_mission_->state_seq == event.seq && latest_mission_->target_id == event.target) {
        return false;
      }
      if (latest_mission_ && !Newer(event.seq, latest_mission_->state_seq)) {
        Invalid("voice photo event has no matching verification session/time window", errors);
        return true;
      }
      return false;
    }
    auto round = session->rounds.find(event.round);
    if (event.event == WAIT_STARTED) {
      if (round != session->rounds.end() || event.round <= session->completed_through) {
        return true;
      }
      if (config_.photos_enabled && !finalizing && latest_frame_stamp_ < event.stamp) {
        return false;
      }
      auto frame = FindFrame(*session, event.stamp);
      if (session->rounds.size() >= config_.frame_history_capacity) {
        ++metrics_.pending_dropped;
        errors.push_back("voice photo round capacity exceeded");
        return true;
      }
      if (config_.photos_enabled && !frame) {
        Missing("voice wait-start photo has no sufficiently recent source frame", errors);
      }
      session->rounds.emplace(event.round, Round{event.stamp, std::move(frame)});
      return true;
    }
    if (round == session->rounds.end()) {
      if (event.round <= session->completed_through) {
        return true;
      }
      return false;
    }
    if (event.stamp < round->second.started_at) {
      Invalid("voice photo result precedes its wait-start event", errors);
      return true;
    }
    if (config_.photos_enabled && event.event == PASSED && !finalizing && latest_frame_stamp_ < event.stamp) {
      return false;
    }
    auto frame = event.event == PASSED ? FindFrame(*session, event.stamp)
                                      : round->second.start_frame;
    const auto started_at = round->second.started_at;
    session->completed_through = std::max(session->completed_through, event.round);
    session->rounds.erase(round);
    if (config_.photos_enabled && !frame) {
      Missing("voice result photo has no matching source frame", errors);
    }
    const VoiceRecord record{
        event, started_at, frame ? std::make_optional(frame->stamp) : std::nullopt,
        !config_.photos_enabled ? "disabled" : (frame ? "saved" : "missing_frame")};
    Enqueue(frame.value_or(Frame{event.stamp, session->lifecycle, event.target, {}}),
            event.event == PASSED ? "voice/passed" : "voice/failed", errors,
            "r" + std::to_string(event.round), record);
    return true;
  }

  void ResolvePending(std::vector<std::string> &errors, bool finalizing = false) {
    // A start later in the queue can unlock a result received ahead of it.
    bool progress;
    do {
      progress = false;
      for (auto it = pending_.begin(); it != pending_.end();) {
        if (Resolve(*it, errors, finalizing)) {
          it = pending_.erase(it);
          progress = true;
        } else {
          ++it;
        }
      }
    } while (progress && !pending_.empty());
  }

  void Enqueue(const Frame &frame, const std::string &directory,
               std::vector<std::string> &errors, const std::string &filename_suffix = std::string{},
               std::optional<VoiceRecord> voice = std::nullopt) {
    if (queue_.size() >= config_.write_queue_capacity) {
      ++metrics_.queue_dropped;
      errors.push_back("photo write queue full; " + directory + " discarded");
      return;
    }
    queue_.push_back({frame, directory, filename_suffix, std::move(voice)});
    ++metrics_.enqueued;
    wake_.notify_one();
  }

  void Missing(const std::string &error, std::vector<std::string> &errors) {
    ++metrics_.missing_frame;
    errors.push_back(error);
  }

  void Invalid(const std::string &error, std::vector<std::string> &errors) {
    ++metrics_.invalid_events;
    errors.push_back(error);
  }

  void Report(const std::vector<std::string> &errors) const noexcept {
    for (const auto &error : errors) {
      try {
        if (error_sink_) {
          error_sink_(error);
        } else {
          std::cerr << "event_photo_recorder: " << error << '\n';
        }
      } catch (...) {
        // Diagnostics must not interrupt perception or the disk worker.
      }
    }
  }

  std::string VoiceJson(const VoiceRecord &record, const std::string &image_file,
                        const std::string &photo_status) const {
    const auto &event = record.event;
    std::ostringstream json;
    // All strings below are fixed labels or the generated ASCII photo basename.
    // No recognized speech, passphrase, or arbitrary path is serialized.
    json << "{\n  \"state_seq\": " << event.seq
         << ",\n  \"target_id\": " << event.target
         << ",\n  \"round_number\": " << event.round
         << ",\n  \"result\": \"" << (event.event == PASSED ? "passed" : "not_passed")
         << "\",\n  \"confidence_valid\": " << (event.confidence ? "true" : "false")
         << ",\n  \"vote_count\": ";
    if (event.confidence) {
      json << static_cast<unsigned>(event.confidence->vote_count);
    } else {
      json << "null";
    }
    json << ",\n  \"matching_channels\": [";
    if (event.confidence) {
      for (std::size_t index = 0; index < event.confidence->matching_channels.size(); ++index) {
        if (index != 0) {
          json << ", ";
        }
        json << static_cast<unsigned>(event.confidence->matching_channels[index]);
      }
    }
    json << "],\n  \"confidence_percent\": ";
    if (event.confidence) {
      json << static_cast<unsigned>(event.confidence->confidence_percent);
    } else {
      json << "null";
    }
    json << ",\n  \"confidence_method\": \"channel_votes_50_75_90_100_v1\""
         << ",\n  \"wait_started_stamp_ns\": " << record.wait_started_stamp
         << ",\n  \"result_stamp_ns\": " << event.stamp
         << ",\n  \"image_stamp_ns\": ";
    if (record.image_stamp) {
      json << *record.image_stamp;
    } else {
      json << "null";
    }
    json << ",\n  \"image_file\": ";
    if (image_file.empty()) {
      json << "null";
    } else {
      json << '"' << image_file << '"';
    }
    json << ",\n  \"photo_status\": \"" << photo_status << "\"\n}\n";
    return json.str();
  }

  std::string Write(const WriteJob &job) {
    for (const char *directory : {"tracking/event", "tracking/lost", "face", "voice", "reid"}) {
      std::filesystem::create_directories(root_ / directory);
    }
    std::filesystem::create_directories(root_ / job.directory);
    const auto base = Filename(job.frame.stamp, job.frame.target) +
                      (job.filename_suffix.empty() ? "" : "_" + job.filename_suffix);
    std::filesystem::path path;
    int fd = -1;
    for (std::uint32_t collision = 0; collision < 100000; ++collision) {
      path = root_ / job.directory /
             (base + (collision == 0 ? "" : "_" + std::to_string(collision)) + ".jpg");
      auto json_path = path;
      json_path.replace_extension(".json");
      if (std::filesystem::exists(json_path) ||
          std::filesystem::exists(std::filesystem::path(json_path.string() + ".tmp"))) {
        continue;
      }
      // The exclusively created JPEG also reserves the basename for JSON-only
      // records until their complete JSON is published.
      fd = ::open(path.c_str(), O_CREAT | O_EXCL | O_WRONLY | O_CLOEXEC, 0600);
      if (fd >= 0) {
        // A metadata-only writer may have published JSON and removed its JPEG
        // reservation between the first existence check and our open.
        if (std::filesystem::exists(json_path)) {
          const int result = ::close(fd);
          const int close_error = errno;
          fd = -1;
          std::error_code ignored;
          std::filesystem::remove(path, ignored);
          if (result != 0) {
            throw std::system_error(close_error, std::generic_category(), "close photo reservation");
          }
          continue;
        }
        break;
      }
      if (errno != EEXIST) {
        throw std::system_error(errno, std::generic_category(), "create photo " + path.string());
      }
    }
    if (fd < 0) {
      throw std::runtime_error("photo filename collision limit exceeded");
    }
    std::string photo_status = job.voice ? job.voice->photo_status : "saved";
    std::string photo_error;
    bool saved = false;
    try {
      if (!job.frame.image.empty()) {
        try {
          std::vector<unsigned char> jpeg;
          if (!cv::imencode(".jpg", job.frame.image, jpeg)) {
            throw std::runtime_error("JPEG encoding failed");
          }
          WriteBytes(fd, jpeg.data(), jpeg.size());
          CloseFile(fd);
          saved = true;
        } catch (const std::exception &error) {
          if (!job.voice) {
            throw;
          }
          photo_status = "write_failed";
          photo_error = std::string("voice photo unavailable; confidence retained: ") + error.what();
        }
      }
      if (fd >= 0) {
        CloseFile(fd);
      }
      if (job.voice) {
        auto json_path = path;
        json_path.replace_extension(".json");
        PublishJson(json_path, VoiceJson(*job.voice, saved ? path.filename().string() : "", photo_status));
      }
      if (!saved) {
        std::error_code ignored;
        std::filesystem::remove(path, ignored);
      }
    } catch (...) {
      if (fd >= 0) {
        ::close(fd);
      }
      std::error_code ignored;
      std::filesystem::remove(path, ignored);
      throw;
    }
    return photo_error;
  }

  void Run() {
    for (;;) {
      WriteJob job;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        wake_.wait(lock, [this] { return stopping_ || !queue_.empty(); });
        if (stopping_ && queue_.empty()) {
          return;
        }
        job = std::move(queue_.front());
        queue_.pop_front();
        writing_ = true;
      }
      std::vector<std::string> errors;
      try {
        const auto photo_error = Write(job);
        std::lock_guard<std::mutex> lock(mutex_);
        ++metrics_.written;
        if (!photo_error.empty()) {
          ++metrics_.write_errors;
          errors.push_back(photo_error);
        }
      } catch (const std::exception &error) {
        std::lock_guard<std::mutex> lock(mutex_);
        ++metrics_.write_errors;
        errors.push_back(std::string("photo write failed: ") + error.what());
      } catch (...) {
        std::lock_guard<std::mutex> lock(mutex_);
        ++metrics_.write_errors;
        errors.push_back("photo write failed: unknown error");
      }
      Report(errors);
      {
        std::lock_guard<std::mutex> lock(mutex_);
        writing_ = false;
      }
      drained_.notify_all();
    }
  }

  Config config_;
  std::filesystem::path root_;
  ErrorSink error_sink_;
  mutable std::mutex mutex_;
  std::condition_variable wake_;
  std::condition_variable drained_;
  std::thread worker_;
  bool stopping_{false};
  bool writing_{false};
  Metrics metrics_;
  std::optional<MissionSnapshot> latest_mission_;
  bool current_mission_available_{false};
  std::uint64_t latest_frame_stamp_{0};
  std::uint64_t lifecycle_{0};
  std::uint64_t lifecycle_started_at_{0};
  int active_target_{0};
  std::deque<Frame> history_;
  std::optional<Frame> last_visible_;
  std::optional<std::pair<std::uint32_t, Frame>> pending_detection_;
  std::deque<Session> sessions_;
  std::deque<VoiceEvent> pending_;
  std::deque<std::tuple<std::uint32_t, int, PerceptionMissionEvent>> recorded_events_;
  std::deque<WriteJob> queue_;
};

EventPhotoRecorder::EventPhotoRecorder(Config config, ErrorSink error_sink)
    : impl_(std::make_unique<Impl>(std::move(config), std::move(error_sink))) {}

EventPhotoRecorder::~EventPhotoRecorder() = default;

void EventPhotoRecorder::ObserveFrame(const std::optional<MissionSnapshot> &mission,
                                      int trusted_target_id,
                                      const std::vector<MissionEventAction> &published_events,
                                      const SourceFrameMetadata &metadata, const cv::Mat &bgr,
                                      std::uint64_t mission_stamp_ns) {
  impl_->ObserveFrame(mission, trusted_target_id, published_events, metadata, bgr, mission_stamp_ns);
}

void EventPhotoRecorder::RecordVoice(std::uint32_t state_seq, int target_id,
                                   std::uint32_t round_number, std::uint8_t event,
                                   std::uint64_t event_stamp_ns,
                                   std::optional<VoiceConfidence> confidence) {
  impl_->RecordVoice(state_seq, target_id, round_number, event, event_stamp_ns, std::move(confidence));
}

void EventPhotoRecorder::Flush() { impl_->Flush(); }

EventPhotoRecorder::Metrics EventPhotoRecorder::GetMetrics() const { return impl_->GetMetrics(); }

}  // namespace dog_patrol_perception_tracking
