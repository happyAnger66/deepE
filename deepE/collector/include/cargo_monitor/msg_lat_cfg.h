#pragma once

#include <pthread.h>
#include <map>
#include <memory>
#include <string>

#include "cargo_monitor_protos/system_monitor_config.pb.h"

namespace cargo {
namespace system_monitor {
class MsgLatCfg {
 public:
  MsgLatCfg();
  static std::shared_ptr<MsgLatCfg> Instance() {
    pthread_once(&once, &MsgLatCfg::init);
    return lat_cfg;
  }

  int32_t GetMsgLatCfg(const std::string &node_name, const std::string &topic,
                       cargo::proto::NodeLatencyConfig &cfg);
  bool IsMsgLatCfg(const std::string &node_name, const std::string &topic);
  const std::map<std::string, cargo::proto::NodeLatencyConfig>
      &GetAllMsgLatCfg() const {
    return all_msg_lats_;
  }
  int32_t GetGlobalMsgLat() { return global_msg_lat_; }
  bool GetGlobalSwitch() { return global_switch_; }
  std::map<std::string, cargo::proto::NodeLatencyConfig> all_msg_lats_;

 private:
  static void init() { lat_cfg = std::make_shared<MsgLatCfg>(); }
  static pthread_once_t once;
  static std::shared_ptr<MsgLatCfg> lat_cfg;

  int32_t global_msg_lat_;
  bool global_switch_ = false;
  std::string get_key(const std::string &node_name, const std::string &topic) {
    return node_name + ":" + topic;
  }
};

}  // namespace system_monitor
}  // namespace cargo
