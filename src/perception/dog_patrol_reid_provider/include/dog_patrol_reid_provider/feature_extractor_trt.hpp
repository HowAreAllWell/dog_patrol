// -*- coding: utf-8 -*-
// TensorRT 后端声明（实现见 feature_extractor_trt.cpp，仅当构建启用
// REID_HAS_TENSORRT 时编入；engine 由 scripts/build_trt_engine.sh 在目标机
// 上用 trtexec --fp16 生成，model_path 指向该 .engine）。
#pragma once

#include <memory>

#include "dog_patrol_reid_provider/feature_extractor.hpp"

namespace dog_patrol_reid_provider {

// 工厂：由 feature_extractor.cpp 的 FeatureExtractor::Create 分发调用。
std::unique_ptr<FeatureExtractor> CreateTrtFeatureExtractor(
    const FeatureExtractor::Config& cfg);

}  // namespace dog_patrol_reid_provider
