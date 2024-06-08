#pragma once

#include <memory>
#include <string>

#include "cargo_monitor_protos/system_info.pb.h"
#include "monitor/monitor_item.h"
#include "shm_stats/shm_cnts_mgr.h"

namespace cargo {
namespace system_monitor {

class NodeLatencyMonitor : public MonitorItem {
 public:
  explicit NodeLatencyMonitor(cargo::proto::SystemMonitorConfig &cfg)
      : MonitorItem(cfg) {}
  int32_t Start(uint32_t interval_ms = 3000) override;
  int32_t RunOnce(cargo::proto::SystemInfo &msg) override;
  int32_t Stop() override;
  const std::string Name() const override { return "NodeLatencyMonitor"; }

 private:
  std::shared_ptr<shm_stats::ShmCntsMgr> shm_cnts_mgr_ = nullptr;
  int32_t add_pub_info(const std::string &node, const std::string &topic,
                       const shm_stats::ShmCntRecord &record,
                       cargo::proto::SystemInfo &msg);
  int32_t add_sub_info(const std::string &node, const std::string &topic,
                       const shm_stats::ShmCntRecord &record,
                       cargo::proto::SystemInfo &msg);
};

}  // namespace system_monitor
}  // namespace cargo
