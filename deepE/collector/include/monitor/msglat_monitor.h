#pragma once

#include <unordered_map>
#include "cargo_monitor/msg_lat_cfg.h"
#include "cargo_monitor_protos/system_info.pb.h"
#include "monitor/monitor_item.h"
#include "shm_stats/shm_latency_mgr.h"

namespace cargo {
namespace system_monitor {

class MsgLatMonitor : public MonitorItem {
 public:
  explicit MsgLatMonitor(cargo::proto::SystemMonitorConfig &cfg)
      : MonitorItem(cfg) {}
  int32_t Start(uint32_t interval_ms = 3000) override;
  int32_t RunOnce(cargo::proto::SystemInfo &msg) override;
  int32_t Stop() override { return 0; }
  const std::string Name() const override { return "MsgLatMonitor"; }

 private:
  int global_latency_ = 100;
  std::shared_ptr<shm_stats::ShmSubLatMgr> shm_sub_lat_mgr_;
  std::shared_ptr<MsgLatCfg> msg_lat_cfg_;
  std::map<std::string, cargo::proto::NodeLatencyConfig> all_msg_cfg_;
  void check_one_topic_msg(const std::string &node, const std::string &topic,
                           cargo::proto::SystemInfo &msg);
};

}  // namespace system_monitor
}  // namespace cargo
