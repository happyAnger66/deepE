#include <glog/logging.h>
#include "base/base_dir.h"

#include "monitor/monitor_config.h"
#include "proto_util/proto_io.h"

namespace cargo {
namespace system_monitor {

pthread_once_t MonitorConfig::once = PTHREAD_ONCE_INIT;
MonitorConfig* MonitorConfig::monitor_config_instance = nullptr;

MonitorConfig::MonitorConfig() {
  // get bring config
  std::string config_path = base::GetBaseDirPath(base::BaseDir::kConfigDir) +
                            "/config/default/onboard/system_monitor.pb.conf";
  LOG(INFO) << "system_monitor config read";
  if (!proto_util::ReadTextProtoFile(config_path, &monitor_cfg_)) {
    LOG(WARNING) << "system_monitor failed to load config: " << config_path;
  }
}

}  // namespace system_monitor
}  // namespace cargo
