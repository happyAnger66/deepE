#include <base/base_dir.h>
#include <glog/logging.h>

#include "av_comm/onboard_config.h"
#include "cargo_monitor/cargo_monitor.h"

namespace cargo {
namespace system_monitor {

bool SystemMonitorGetConfig(cargo::proto::SystemMonitorConfig &config) {
  config = av_comm::CreateConfig<cargo::proto::SystemMonitorConfig>(
      "system_monitor_config");
  return true;
}

}  // namespace system_monitor
}  // namespace cargo
