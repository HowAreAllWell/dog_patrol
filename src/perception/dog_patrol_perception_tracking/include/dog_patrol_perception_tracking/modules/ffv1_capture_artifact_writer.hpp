#pragma once

#include "dog_patrol_perception_tracking/modules/ffv1_capture_workflow.hpp"

#include <filesystem>
#include <memory>
#include <string>

namespace dog_patrol_perception_tracking {

class Ffv1CaptureArtifactWriterFactory final : public CaptureArtifactWriterFactory {
 public:
  struct Config {
    std::filesystem::path session_directory;
    double requested_fps{30.0};
    std::string mvs_model;
    std::string mvs_serial;
    int requested_width{1280};
    int requested_height{1024};
    int timeout_ms{1000};
  };

  explicit Ffv1CaptureArtifactWriterFactory(Config config);
  std::unique_ptr<CaptureArtifactWriter> Create() override;

 private:
  Config config_;
};

}  // namespace dog_patrol_perception_tracking
