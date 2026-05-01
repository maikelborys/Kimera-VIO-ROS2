/**
 * @file   KimeraVioRosNode.cpp
 * @brief  Standalone ROS 2 entry point for the Kimera-VIO wrapper.
 *
 * gflags + rclcpp::init coexistence: gflags consumes its --foo args first,
 * rclcpp::init then handles the remaining --ros-args.
 */

#include <memory>
#include <thread>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <rclcpp/rclcpp.hpp>

#include "kimera_vio_ros/KimeraVioRos.hpp"

int main(int argc, char* argv[]) {
  // gflags trips on `--ros-args ...`. Hand it only the prefix of argv that
  // ends just before `--ros-args`; pass the full argv to rclcpp::init so it
  // can consume its own args from anywhere.
  int gflags_argc = argc;
  for (int i = 1; i < argc; ++i) {
    if (std::string(argv[i]) == "--ros-args") {
      gflags_argc = i;
      break;
    }
  }
  char** gflags_argv = argv;
  gflags::ParseCommandLineFlags(&gflags_argc, &gflags_argv,
                                /*remove_flags=*/false);

  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  FLAGS_logtostderr = true;
  FLAGS_colorlogtostderr = true;
  FLAGS_log_prefix = true;

  rclcpp::init(argc, argv);

  rclcpp::NodeOptions opts;
  opts.allow_undeclared_parameters(true);
  opts.automatically_declare_parameters_from_overrides(true);

  auto kimera = std::make_shared<VIO::KimeraVioRos>(opts);

  // Multi-threaded executor so the IMU reentrant callback group can run in
  // parallel with the stereo image-sync callback group.
  rclcpp::executors::MultiThreadedExecutor executor(
      rclcpp::ExecutorOptions(),
      /*num_threads=*/std::max<size_t>(2u,
          std::thread::hardware_concurrency() / 2));
  executor.add_node(kimera->node());

  // Pipeline runs in its own thread; the executor keeps the callbacks live.
  std::thread vio_thread([&kimera]() {
    try {
      kimera->runKimeraVio();
    } catch (const std::exception& e) {
      LOG(ERROR) << "Kimera-VIO threw: " << e.what();
    }
    rclcpp::shutdown();
  });

  executor.spin();
  if (vio_thread.joinable()) {
    kimera->shutdown();
    vio_thread.join();
  }
  rclcpp::shutdown();
  return 0;
}
