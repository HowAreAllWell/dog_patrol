#include "vision_demo_host/modules/camera_ingest.hpp"
#include "vision_demo_host/modules/ffv1_capture_artifact_writer.hpp"
#include "vision_demo_host/modules/ffv1_capture_workflow.hpp"

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <chrono>
#include <cctype>
#include <csignal>
#include <cstdint>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

namespace {

volatile std::sig_atomic_t g_interrupted = 0;

void SigintHandler(int) { g_interrupted = 1; }

struct Options {
  std::string mvs_model{"MV-CU013-A0UC"};
  std::string mvs_serial;
  int width{1280};
  int height{1024};
  double fps{30.0};
  int timeout_ms{1000};
  vision_demo_host::CameraIngest::BayerInterpolation bayer_interpolation{
      vision_demo_host::CameraIngest::kDefaultBayerInterpolation};
  bool bayer_smoothing{vision_demo_host::CameraIngest::kDefaultBayerSmoothing};
  std::string output_root{"data/captures"};
  std::string session_name{"capture"};
  std::size_t queue_capacity{120};
  bool headless{false};
  bool auto_record{false};
  int max_seconds{0};
};

std::uint64_t WallTimeNs() {
  return static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::system_clock::now().time_since_epoch())
          .count());
}

std::string TimestampNowCompact() {
  const std::time_t now = std::time(nullptr);
  std::tm local_time{};
  localtime_r(&now, &local_time);
  std::ostringstream timestamp;
  timestamp << std::put_time(&local_time, "%Y%m%d_%H%M%S");
  return timestamp.str();
}

const char *StateName(const vision_demo_host::CaptureState state) {
  switch (state) {
    case vision_demo_host::CaptureState::kStandby:
      return "STANDBY";
    case vision_demo_host::CaptureState::kRecording:
      return "RECORDING";
    case vision_demo_host::CaptureState::kFinalizing:
      return "FINALIZING";
    case vision_demo_host::CaptureState::kStopped:
      return "STOPPED";
  }
  return "UNKNOWN";
}

void PrintUsage() {
  std::cout
      << "Usage: capture_ffv1 [options]\n"
      << "  Records clean BGR8 frames from one Hik MVS camera as mandatory FFV1/MKV takes.\n"
      << "  Interactive controls: R=start take, S=stop/finalize, M=marker, I=toggle info, Q=finalize/quit.\n"
      << "  --mvs-model <model>                  (default: MV-CU013-A0UC)\n"
      << "  --mvs-serial <serial>\n"
      << "  --width <px>                         (default: 1280)\n"
      << "  --height <px>                        (default: 1024)\n"
      << "  --fps <fps>                          (default: 30.0)\n"
      << "  --timeout-ms <ms>                    (default: 1000)\n"
      << "  --bayer-interpolation <fast|balanced|optimal|optimal_plus> (default: balanced)\n"
      << "  --bayer-smoothing                    (default: disabled)\n"
      << "  --output-root <dir>                  (default: data/captures)\n"
      << "  --session-name <name>                (default: capture)\n"
      << "  --queue-capacity <frames>            (default: 120; full queue drops newest frames)\n"
      << "  --headless                           (automation mode; no preview)\n"
      << "  --auto-record                        (start a take immediately)\n"
      << "  --max-seconds <n>                    (required in headless mode)\n"
      << "  --help\n";
}

bool ParsePositiveInt(const std::string &value, const std::string &name, int *out,
                      std::string *error) {
  try {
    *out = std::stoi(value);
  } catch (...) {
    if (error != nullptr) {
      *error = "Invalid integer for " + name + ": " + value;
    }
    return false;
  }
  if (*out <= 0) {
    if (error != nullptr) {
      *error = name + " must be positive";
    }
    return false;
  }
  return true;
}

bool ParseArgs(const int argc, char **argv, Options *options, std::string *error) {
  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];
    const auto value = [&](const std::string &name) -> std::string {
      if (index + 1 >= argc) {
        if (error != nullptr) {
          *error = "Missing value for " + name;
        }
        return "";
      }
      return argv[++index];
    };
    if (argument == "--mvs-model") {
      options->mvs_model = value(argument);
    } else if (argument == "--mvs-serial") {
      options->mvs_serial = value(argument);
    } else if (argument == "--width") {
      if (!ParsePositiveInt(value(argument), argument, &options->width, error)) {
        return false;
      }
    } else if (argument == "--height") {
      if (!ParsePositiveInt(value(argument), argument, &options->height, error)) {
        return false;
      }
    } else if (argument == "--fps") {
      try {
        options->fps = std::stod(value(argument));
      } catch (...) {
        if (error != nullptr) {
          *error = "Invalid number for --fps";
        }
        return false;
      }
    } else if (argument == "--timeout-ms") {
      if (!ParsePositiveInt(value(argument), argument, &options->timeout_ms, error)) {
        return false;
      }
    } else if (argument == "--bayer-interpolation") {
      if (!vision_demo_host::CameraIngest::ParseBayerInterpolation(
              value(argument), &options->bayer_interpolation, error)) {
        return false;
      }
    } else if (argument == "--bayer-smoothing") {
      options->bayer_smoothing = true;
    } else if (argument == "--output-root") {
      options->output_root = value(argument);
    } else if (argument == "--session-name") {
      options->session_name = value(argument);
    } else if (argument == "--queue-capacity") {
      int parsed_capacity = 0;
      if (!ParsePositiveInt(value(argument), argument, &parsed_capacity, error)) {
        return false;
      }
      options->queue_capacity = static_cast<std::size_t>(parsed_capacity);
    } else if (argument == "--headless") {
      options->headless = true;
    } else if (argument == "--auto-record") {
      options->auto_record = true;
    } else if (argument == "--max-seconds") {
      if (!ParsePositiveInt(value(argument), argument, &options->max_seconds, error)) {
        return false;
      }
    } else if (argument == "--help" || argument == "-h") {
      PrintUsage();
      return false;
    } else {
      if (error != nullptr) {
        *error = "Unknown argument: " + argument;
      }
      return false;
    }
    if (error != nullptr && !error->empty()) {
      return false;
    }
  }

  if (options->headless && !options->auto_record) {
    if (error != nullptr) {
      *error = "--headless requires --auto-record";
    }
    return false;
  }
  if (options->headless && options->max_seconds <= 0) {
    if (error != nullptr) {
      *error = "--headless requires --max-seconds > 0";
    }
    return false;
  }
  vision_demo_host::CameraIngest::Config camera_config;
  camera_config.width = options->width;
  camera_config.height = options->height;
  camera_config.fps = options->fps;
  camera_config.timeout_ms = options->timeout_ms;
  camera_config.bayer_interpolation = options->bayer_interpolation;
  camera_config.bayer_smoothing = options->bayer_smoothing;
  return vision_demo_host::CameraIngest::ValidateConfig(camera_config, error);
}

void DrawPreview(cv::Mat *preview, const vision_demo_host::CaptureSnapshot &snapshot,
                 const vision_demo_host::CameraIngest::AcquiredFrame &frame,
                 const double capture_fps) {
  if (preview == nullptr || preview->empty() || !snapshot.preview_info_enabled) {
    return;
  }
  const vision_demo_host::CaptureTakeSummary &take = snapshot.active_take;
  const cv::Scalar state_color = snapshot.state == vision_demo_host::CaptureState::kRecording
                                     ? cv::Scalar(0, 0, 255)
                                     : cv::Scalar(0, 255, 0);
  const std::string source_format = frame.source_pixel_type_name.empty()
                                        ? vision_demo_host::CameraIngest::PixelTypeName(
                                              frame.source_pixel_type)
                                        : frame.source_pixel_type_name;
  const std::string lines[] = {
      std::string("STATE: ") + StateName(snapshot.state) +
          (snapshot.active_take_name.empty() ? "" : "  TAKE: " + snapshot.active_take_name),
      "CAPTURE: " + std::to_string(capture_fps) + " FPS  " +
          std::to_string(frame.bgr8.cols) + "x" + std::to_string(frame.bgr8.rows) +
          "  SOURCE: " + source_format,
      "FRAMES captured/written/dropped: " + std::to_string(take.captured_frames) + "/" +
          std::to_string(take.written_frames) + "/" + std::to_string(take.dropped_frames),
      "WRITE_ERRORS: " + std::to_string(take.write_errors) +
          "  KEYS: R start, S stop, M marker, I info, Q quit",
  };
  for (std::size_t index = 0; index < std::size(lines); ++index) {
    cv::putText(*preview, lines[index], cv::Point(16, 30 + static_cast<int>(28 * index)),
                cv::FONT_HERSHEY_SIMPLEX, 0.55,
                index == 0U ? state_color : cv::Scalar(255, 255, 0), 1, cv::LINE_AA);
  }
}

void PrintTakeSummaries(const vision_demo_host::CaptureSnapshot &snapshot) {
  for (const vision_demo_host::CaptureTakeSummary &take : snapshot.completed_takes) {
    std::cout << "[capture] take=" << take.descriptor.name
              << " state=" << (take.complete ? "complete" : "incomplete")
              << " captured=" << take.captured_frames << " written=" << take.written_frames
              << " dropped=" << take.dropped_frames << " write_errors=" << take.write_errors
              << " camera_frame_gaps=" << take.camera_frame_gaps << std::endl;
  }
}

}  // namespace

int main(int argc, char **argv) {
  std::signal(SIGINT, SigintHandler);
  Options options;
  std::string error;
  if (!ParseArgs(argc, argv, &options, &error)) {
    if (!error.empty()) {
      std::cerr << "Argument error: " << error << std::endl;
      PrintUsage();
      return 2;
    }
    return 0;
  }
  if (!options.headless) {
    const char *display = std::getenv("DISPLAY");
    if (display == nullptr || std::string(display).empty()) {
      std::cerr << "Preview-first mode requires DISPLAY. For automation use --headless "
                   "--auto-record --max-seconds <n>."
                << std::endl;
      return 3;
    }
  }

  vision_demo_host::CameraIngest::Config camera_config;
  camera_config.hik_mvs_model = options.mvs_model;
  camera_config.hik_mvs_serial = options.mvs_serial;
  camera_config.width = options.width;
  camera_config.height = options.height;
  camera_config.fps = options.fps;
  camera_config.timeout_ms = options.timeout_ms;
  camera_config.bayer_interpolation = options.bayer_interpolation;
  camera_config.bayer_smoothing = options.bayer_smoothing;
  vision_demo_host::CameraIngest camera;
  if (!camera.Open(camera_config, &error)) {
    std::cerr << "Failed to open Hik MVS camera: " << error << std::endl;
    return 4;
  }

  const std::filesystem::path session_directory =
      std::filesystem::path(options.output_root) /
      (options.session_name + "_" + TimestampNowCompact());
  vision_demo_host::Ffv1CaptureArtifactWriterFactory::Config writer_config;
  writer_config.session_directory = session_directory;
  writer_config.requested_fps = options.fps;
  writer_config.mvs_model = options.mvs_model;
  writer_config.mvs_serial = options.mvs_serial;
  writer_config.requested_width = options.width;
  writer_config.requested_height = options.height;
  writer_config.timeout_ms = options.timeout_ms;
  vision_demo_host::Ffv1CaptureWorkflow::Config workflow_config;
  workflow_config.take_name_prefix = "take";
  workflow_config.queue_capacity = options.queue_capacity;
  workflow_config.bayer_interpolation = options.bayer_interpolation;
  workflow_config.bayer_smoothing = options.bayer_smoothing;
  vision_demo_host::Ffv1CaptureWorkflow workflow(
      workflow_config,
      std::make_unique<vision_demo_host::Ffv1CaptureArtifactWriterFactory>(writer_config));

  if (options.auto_record &&
      !workflow.HandleControl(vision_demo_host::CaptureControl::kStart, WallTimeNs(), &error)) {
    std::cerr << "Failed to start automatic take: " << error << std::endl;
    camera.Close();
    return 5;
  }
  const auto loop_started = std::chrono::steady_clock::now();
  std::uint64_t acquired_frames = 0;
  bool quit_requested = false;
  bool interrupted = false;
  int exit_code = 0;
  const std::string preview_window = "capture_ffv1_preview";
  if (!options.headless) {
    cv::namedWindow(preview_window, cv::WINDOW_NORMAL);
    std::cout << "[capture] preview ready. R=start, S=stop/finalize, M=marker, I=toggle info, "
                 "Q=finalize/quit."
              << std::endl;
  }

  while (!g_interrupted && !quit_requested) {
    vision_demo_host::CameraIngest::AcquiredFrame frame;
    if (!camera.Read(&frame, &error) || frame.bgr8.empty()) {
      std::cerr << "Hik MVS frame acquisition failed: " << error << std::endl;
      interrupted = true;
      exit_code = 6;
      break;
    }
    ++acquired_frames;
    const auto snapshot_before_submit = workflow.Snapshot();
    if (snapshot_before_submit.state == vision_demo_host::CaptureState::kRecording) {
      workflow.Submit(frame);
    }
    const auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - loop_started);
    const double capture_fps = elapsed.count() > 0.0
                                   ? static_cast<double>(acquired_frames) / elapsed.count()
                                   : 0.0;
    const auto snapshot = workflow.Snapshot();
    if (snapshot.state == vision_demo_host::CaptureState::kRecording &&
        !snapshot.active_take.last_write_error.empty()) {
      std::cerr << "FFV1 write failure: " << snapshot.active_take.last_write_error << std::endl;
      interrupted = true;
      exit_code = 7;
      break;
    }

    if (!options.headless) {
      cv::Mat preview = frame.bgr8.clone();
      DrawPreview(&preview, snapshot, frame, capture_fps);
      cv::imshow(preview_window, preview);
      const int key = cv::waitKey(1) & 0xFF;
      const int normalized_key = std::toupper(key);
      vision_demo_host::CaptureControl control{};
      bool has_control = true;
      switch (normalized_key) {
        case 'R':
          control = vision_demo_host::CaptureControl::kStart;
          break;
        case 'S':
          control = vision_demo_host::CaptureControl::kStop;
          break;
        case 'M':
          control = vision_demo_host::CaptureControl::kAddMarker;
          break;
        case 'I':
          control = vision_demo_host::CaptureControl::kTogglePreviewInfo;
          break;
        case 'Q':
          control = vision_demo_host::CaptureControl::kQuit;
          break;
        default:
          has_control = false;
          break;
      }
      if (has_control && !workflow.HandleControl(control, WallTimeNs(), &error)) {
        std::cerr << "Capture control failed: " << error << std::endl;
      }
      quit_requested = has_control && control == vision_demo_host::CaptureControl::kQuit;
    }

    if (options.auto_record && options.max_seconds > 0 &&
        elapsed >= std::chrono::seconds(options.max_seconds)) {
      if (!workflow.HandleControl(vision_demo_host::CaptureControl::kStop, WallTimeNs(), &error)) {
        std::cerr << "Failed to finalize automatic take: " << error << std::endl;
        exit_code = 8;
      }
      quit_requested = true;
    }
  }

  if (g_interrupted || interrupted) {
    workflow.Interrupt(WallTimeNs());
  } else if (!quit_requested && workflow.Snapshot().state == vision_demo_host::CaptureState::kRecording) {
    workflow.Interrupt(WallTimeNs());
  }
  camera.Close();
  if (!options.headless) {
    cv::destroyWindow(preview_window);
  }
  const auto final_snapshot = workflow.Snapshot();
  PrintTakeSummaries(final_snapshot);
  std::cout << "[capture] session directory: " << session_directory << std::endl;
  return exit_code;
}
