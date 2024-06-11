#pragma once

#include <pthread.h>
#include <map>
#include <memory>
#include <string>

#include "cargo_monitor_protos/system_monitor_config.pb.h"

namespace cargo {
namespace system_monitor {

class TopicCfg {
 public:
  TopicCfg();
  static std::shared_ptr<TopicCfg> Instance() {
    pthread_once(&once, &TopicCfg::init);
    return topic_cfg;
  }

  int32_t GetTopicCfg(const std::string &topic, cargo::proto::TopicConfig &cfg);
  const std::map<std::string, cargo::proto::TopicConfig> &GetAllTopicCfgs() {
    return all_topics;
  }
  std::map<std::string, cargo::proto::TopicConfig> all_topics;

 private:
  static void init() { topic_cfg = std::make_shared<TopicCfg>(); }
  static pthread_once_t once;
  static std::shared_ptr<TopicCfg> topic_cfg;
};

}  // namespace system_monitor
}  // namespace cargo
