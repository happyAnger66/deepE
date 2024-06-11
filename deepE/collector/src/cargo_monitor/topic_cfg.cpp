#include <pthread.h>
#include <map>
#include <memory>
#include <string>

#include "cargo_monitor_protos/system_monitor_config.pb.h"
#include "base/base_dir.h"
#include "absl/strings/str_format.h"
#include "av_comm/onboard_config.h"
#include "cargo_monitor/topic_cfg.h"
#include "proto_util/proto_io.h"

namespace cargo {
namespace system_monitor {

TopicCfg::TopicCfg() {
  cargo::proto::SystemMonitorConfig system_monitor_pb_config;
  system_monitor_pb_config =
      av_comm::CreateConfig<cargo::proto::SystemMonitorConfig>(
          "system_monitor_config");

  for (auto &topic : system_monitor_pb_config.topic()) {
    all_topics[topic.name()] = topic;
  }
}

int32_t TopicCfg::GetTopicCfg(const std::string &topic,
                              cargo::proto::TopicConfig &cfg) {
  if (all_topics.find(topic) == all_topics.end()) {
    return -1;
  }

  cfg = all_topics[topic];
  return 0;
}

pthread_once_t TopicCfg::once = PTHREAD_ONCE_INIT;
std::shared_ptr<TopicCfg> TopicCfg::topic_cfg = nullptr;

}  // namespace system_monitor
}  // namespace cargo
