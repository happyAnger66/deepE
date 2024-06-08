#include <pthread.h>
#include <map>
#include <memory>
#include <string>

#include "cargo_monitor_protos/system_monitor_config.pb.h"
#include "base/base_dir.h"
#include "absl/strings/str_format.h"
#include "av_comm/onboard_config.h"
#include "cargo_monitor/msg_lat_cfg.h"
#include "proto_util/proto_io.h"

namespace cargo {
namespace system_monitor {

MsgLatCfg::MsgLatCfg() {
  cargo::proto::SystemMonitorConfig system_monitor_pb_config;
  system_monitor_pb_config =
      av_comm::CreateConfig<cargo::proto::SystemMonitorConfig>(
          "system_monitor_config");

  for (auto &one_msg_lat : system_monitor_pb_config.node_msg_lat()) {
    all_msg_lats_[get_key(one_msg_lat.node(), one_msg_lat.topic())] =
        one_msg_lat;
  }

  global_msg_lat_ = system_monitor_pb_config.msg_lat_cfg().latency();
  global_switch_ = system_monitor_pb_config.msg_lat_cfg().global_switch();
}

bool MsgLatCfg::IsMsgLatCfg(const std::string &node_name,
                            const std::string &topic) {
  if (all_msg_lats_.find(get_key(node_name, topic)) == all_msg_lats_.end()) {
    return false;
  }

  return true;
}

int32_t MsgLatCfg::GetMsgLatCfg(const std::string &node_name,
                                const std::string &topic,
                                cargo::proto::NodeLatencyConfig &cfg) {
  std::string key = get_key(node_name, topic);
  if (all_msg_lats_.find(key) == all_msg_lats_.end()) {
    return -1;
  }

  cfg = all_msg_lats_[key];
  return 0;
}

pthread_once_t MsgLatCfg::once = PTHREAD_ONCE_INIT;
std::shared_ptr<MsgLatCfg> MsgLatCfg::lat_cfg = nullptr;

}  // namespace system_monitor
}  // namespace cargo
