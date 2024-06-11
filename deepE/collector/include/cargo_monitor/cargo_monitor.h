#pragma once

#include <string>
#include <unordered_map>
#include "cargo_monitor_protos/system_monitor_config.pb.h"

namespace cargo {
namespace system_monitor {

struct OneTopicHz {
  std::string name;
  double hz_normal;
  double hz_cur;
};

using TopicHzInfo = std::unordered_map<std::string, OneTopicHz>;
bool SystemMonitorGetConfig(cargo::proto::SystemMonitorConfig &config);
bool SystemMonitorGetAllTopicsHz(TopicHzInfo &topic_hz_info);

}  // namespace system_monitor
}  // namespace cargo
