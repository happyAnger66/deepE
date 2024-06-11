#include "system_monitor_node.h"

#include <base/base_dir.h>
#include <base/now.h>
#include <glog/logging.h>

#include "log/async_logsink_entry.h"
#include "tracing/tracer.h"
#include "utils/ros_utils.h"

int main(int argc, char **argv) {
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  cargo::common::logsink::InitAsyncLogSink(argv[0]);

  // get module config
  using namespace cargo::system_monitor;

  /*RosUtils::RosRole role = RosUtils::GetRole();
  LOG(INFO) << "cargo_system_monitor role: ["
            << std::to_string(static_cast<int>(role)).c_str() << "]";*/

  std::string config_file;
  if (!RosUtils::GetNodeConfigFile(config_file)) {
    LOG(ERROR) << "cargo_monitor get node_cfg_file failed !!!";
    return -1;
  }

  cargo::system_monitor::SystemMonitorNode node(config_file);
  auto tracer = tracing::Tracer::instance();
  tracer->Init(node.name());
  tracer->Start();

  node.Init();
  node.Start();

  //  ros::spin();
  cargo::WaitForShutdown();
}
