#pragma once

#include "monitor/monitor_item.h"

namespace cargo {
namespace system_monitor {

class MemMonitor : public MonitorItem {
 public:
  explicit MemMonitor(cargo::proto::SystemMonitorConfig &cfg)
      : MonitorItem(cfg) {}
  int32_t Start(uint32_t interval_ms = 3000) override { return 0; }
  int32_t RunOnce(cargo::proto::SystemInfo &system_info_msg) override;
  int32_t Stop() override { return 0; }
  const std::string Name() const override { return "MemMonitor"; }
};
}  // namespace system_monitor
}  // namespace cargo
