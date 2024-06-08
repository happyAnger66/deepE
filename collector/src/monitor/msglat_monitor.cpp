#include <iomanip>
#include <vector>

#include <absl/strings/str_split.h>
#include <glog/logging.h>

#include "common_protos/status.pb.h"
#include "monitor/msglat_monitor.h"
#include "shm_stats/shm_hz.h"
#include "utils/ros_utils.h"

namespace cargo {
namespace system_monitor {

static constexpr int KUsToMs = 1000;

int32_t MsgLatMonitor::Start(uint32_t interval_ms) {
  msg_lat_cfg_ = MsgLatCfg::Instance();
  shm_sub_lat_mgr_ = shm_stats::ShmSubLatMgr::Instance();

  global_latency_ = msg_lat_cfg_->GetGlobalMsgLat();

  (void)interval_ms;
  return 0;
}

int32_t MsgLatMonitor::RunOnce(cargo::proto::SystemInfo &msg) {
  ros::V_string local_nodes;
  if (!RosUtils::GetAllLocalNodes(local_nodes)) {
    return -1;
  }

  for (auto &node : local_nodes) {
    std::set<std::string> topics;
    if (!RosUtils::GetNodeSubTopics(node, topics)) {
      continue;
    }

    for (auto &topic : topics) {
      check_one_topic_msg(node, topic, msg);
    }
  }

  return 0;
}

void MsgLatMonitor::check_one_topic_msg(const std::string &node,
                                        const std::string &topic,
                                        cargo::proto::SystemInfo &msg) {
  int latency = global_latency_;
  cargo::proto::NodeLatencyConfig cfg;
  static bool global_switch = msg_lat_cfg_->GetGlobalSwitch();

  if (msg_lat_cfg_->GetMsgLatCfg(node, topic, cfg) == 0) {
    latency = cfg.latency();
  } else if (!global_switch) {
    return;
  }

  if (latency <= 0) {
    latency = 100;
  }

  shm_stats::LatInfo lat_info;
  shm_stats::LatStatus status =
      shm_sub_lat_mgr_->GetLatInfo(node, topic, lat_info);
  if (status == shm_stats::LatStatus::NORMAL) {
    auto sub_lat_info = msg.add_sublat_info();
    sub_lat_info->set_node(node);
    sub_lat_info->set_topic(topic);
    sub_lat_info->set_latency(lat_info.rate);

    if (lat_info.last_time / KUsToMs > latency) {
      auto sub_lat_err = msg.add_sublat_err();
      sub_lat_err->set_node(node);
      sub_lat_err->set_topic(topic);
      sub_lat_err->set_latency(lat_info.last_time / KUsToMs);
      sub_lat_err->set_normal(latency);

      std::stringstream ss;
      auto one_status = msg.add_status_list();

      ss << "node: " << node << " topic: " << topic << " last latency [ "
         << std::setprecision(3) << lat_info.last_time / KUsToMs
         << " ]ms is greater than [ " << latency << " ]ms ";

      one_status->set_msg(ss.str());
      one_status->set_error_code(cargo::common::ErrorCode::TOPIC_LAT_ERROR);
    }
  }

  return;
}

}  // namespace system_monitor
}  // namespace cargo
