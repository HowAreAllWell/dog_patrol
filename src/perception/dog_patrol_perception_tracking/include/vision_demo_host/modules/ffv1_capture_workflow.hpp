#pragma once

#include "vision_demo_host/modules/camera_ingest.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace vision_demo_host {

enum class CaptureState : unsigned int {
  kStandby,
  kRecording,
  kFinalizing,
  kStopped,
};

enum class CaptureControl : unsigned int {
  kStart,
  kStop,
  kAddMarker,
  kTogglePreviewInfo,
  kQuit,
};

struct CaptureFrameContract {
  std::string output_pixel_format{"BGR8"};
  std::uint32_t source_pixel_type{0};
  std::string source_pixel_type_name;
  int width{0};
  int height{0};
  std::size_t source_payload_bytes{0};
  std::string bayer_interpolation;
  bool bayer_smoothing{false};
};

struct CaptureTakeDescriptor {
  unsigned int sequence{0};
  std::string name;
  std::uint64_t started_wall_time_ns{0};
};

struct CaptureFrame {
  std::uint64_t capture_index{0};
  CameraIngest::AcquiredFrame source;
};

struct CaptureMarker {
  unsigned int index{0};
  std::uint64_t wall_time_ns{0};
  double elapsed_seconds{0.0};
  std::string note;
};

struct CaptureTakeSummary {
  CaptureTakeDescriptor descriptor;
  CaptureFrameContract frame_contract;
  bool complete{false};
  bool writer_opened{false};
  std::uint64_t finished_wall_time_ns{0};
  std::uint64_t captured_frames{0};
  std::uint64_t written_frames{0};
  std::uint64_t dropped_frames{0};
  std::uint64_t write_errors{0};
  std::uint64_t camera_frame_gaps{0};
  std::string last_write_error;
  std::vector<CaptureMarker> markers;
};

struct CaptureSnapshot {
  CaptureState state{CaptureState::kStandby};
  bool preview_info_enabled{true};
  std::string active_take_name;
  CaptureTakeSummary active_take;
  std::vector<CaptureTakeSummary> completed_takes;
};

class CaptureArtifactWriter {
 public:
  virtual ~CaptureArtifactWriter() = default;

  virtual bool Begin(const CaptureTakeDescriptor &descriptor,
                     const CaptureFrameContract &frame_contract,
                     std::string *error) = 0;
  virtual bool Write(const CaptureFrame &frame, std::string *error) = 0;
  virtual bool Finish(const CaptureTakeSummary &summary, std::string *error) = 0;
};

class CaptureArtifactWriterFactory {
 public:
  virtual ~CaptureArtifactWriterFactory() = default;

  virtual std::unique_ptr<CaptureArtifactWriter> Create() = 0;
};

class Ffv1CaptureWorkflow {
 public:
  struct Config {
    std::string take_name_prefix{"take"};
    std::size_t queue_capacity{120};
    CameraIngest::BayerInterpolation bayer_interpolation{
        CameraIngest::BayerInterpolation::kOptimal};
    bool bayer_smoothing{false};
  };

  Ffv1CaptureWorkflow(Config config,
                      std::unique_ptr<CaptureArtifactWriterFactory> writer_factory);
  ~Ffv1CaptureWorkflow();

  Ffv1CaptureWorkflow(const Ffv1CaptureWorkflow &) = delete;
  Ffv1CaptureWorkflow &operator=(const Ffv1CaptureWorkflow &) = delete;

  bool HandleControl(CaptureControl control, std::uint64_t wall_time_ns,
                     std::string *error);
  void Submit(const CameraIngest::AcquiredFrame &frame);
  void Interrupt(std::uint64_t wall_time_ns);
  CaptureSnapshot Snapshot() const;

 private:
  struct ActiveTake;

  static CaptureFrameContract ContractFrom(const CameraIngest::AcquiredFrame &frame,
                                           const Config &config);
  static std::uint64_t CountCameraFrameGap(const CameraIngest::AcquiredFrame &previous,
                                           const CameraIngest::AcquiredFrame &current);
  bool Start(std::uint64_t wall_time_ns, std::string *error);
  bool Finalize(bool complete, bool stop_after_finalize, std::uint64_t wall_time_ns,
                std::string *error);
  static void RunWriter(const std::shared_ptr<ActiveTake> &take, const Config &config);

  Config config_;
  std::unique_ptr<CaptureArtifactWriterFactory> writer_factory_;
  mutable std::mutex mutex_;
  CaptureState state_{CaptureState::kStandby};
  bool preview_info_enabled_{true};
  unsigned int next_take_sequence_{1};
  std::shared_ptr<ActiveTake> active_take_;
  std::vector<CaptureTakeSummary> completed_takes_;
};

}  // namespace vision_demo_host
