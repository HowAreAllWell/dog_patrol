#include <iostream>
#include <string>
#include <utility>

#include "dog_patrol_perception_tracking/modules/preprocess_infer.hpp"

int main(int argc, char **argv) {
  if (argc != 2) {
    std::cerr << "usage: validate_tensorrt_engine <model.engine>" << std::endl;
    return 2;
  }

  dog_patrol_perception_tracking::PreprocessInfer::Config config;
  config.detector_runtime_path = argv[1];
  config.enable_fake_detection = false;
  dog_patrol_perception_tracking::PreprocessInfer detector(std::move(config));
  std::string error;
  if (!detector.Initialize(&error)) {
    std::cerr << "production TensorRT runtime could not load " << argv[1] << ": "
              << error << std::endl;
    return 1;
  }

  std::cout << "TENSORRT_ENGINE_LOAD_OK " << argv[1] << std::endl;
  return 0;
}
