#pragma once

#include <memory>
#include <string>
#include <unordered_map>

#include "cargo_monitor_protos/topics_graph.pb.h"
#include "av_comm/onboard_config.h"
#include "av_comm/singleton.h"

namespace cargo {
namespace system_monitor {

class TopicsGraphCfg : public av_comm::Singleton<TopicsGraphCfg> {
  friend class av_comm::Singleton<TopicsGraphCfg>;

 public:
  std::string FindInputTopic(const std::string &topic) {
    const auto &iter = topic_graph_.find(topic);
    if (iter != topic_graph_.end()) {
      return iter->second;
    }

    return "";
  }

 private:
  TopicsGraphCfg() {
    topic_graph_pb_ =
        av_comm::CreateConfig<cargo::common::TopicGraph>("topics_graph");
    for (auto &edge : topic_graph_pb_.edge()) {
      topic_graph_[edge.topic()] = edge.input_topic();
    }
  }
  cargo::common::TopicGraph topic_graph_pb_;
  std::unordered_map<std::string, std::string> topic_graph_;
};

}  // namespace system_monitor
}  // namespace cargo
