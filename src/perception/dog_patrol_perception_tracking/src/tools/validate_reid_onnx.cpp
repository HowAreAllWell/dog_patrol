#include <exception>
#include <iostream>

#include <opencv2/dnn.hpp>

int main(int argc, char **argv) {
  if (argc != 2) {
    std::cerr << "usage: validate_reid_onnx <model.onnx>" << std::endl;
    return 2;
  }
  try {
    const cv::dnn::Net network = cv::dnn::readNetFromONNX(argv[1]);
    if (network.empty()) {
      std::cerr << "OpenCV DNN returned an empty network for " << argv[1] << std::endl;
      return 1;
    }
  } catch (const std::exception &exception) {
    std::cerr << "OpenCV DNN could not load " << argv[1] << ": " << exception.what()
              << std::endl;
    return 1;
  }
  std::cout << "ONNX_LOAD_OK " << argv[1] << std::endl;
  return 0;
}

