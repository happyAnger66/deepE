#pragma once
#include <boost/chrono.hpp>

#include "monitor/monitor_item.h"
#include "utils/libsensors_chip.h"
#include "utils/proc_stat.h"

namespace cargo {
namespace system_monitor {

class CpuTempMonitor : public MonitorItem {
 public:
  explicit CpuTempMonitor(cargo::proto::SystemMonitorConfig &cfg)
      : MonitorItem(cfg) {}
  int32_t Start(uint32_t interval_ms = 3000) override;
  int32_t RunOnce(cargo::proto::SystemInfo &msg) override;
  int32_t Stop() override;
  const std::string Name() const override { return "CpuTempMonitor"; }

 private:
  // All of the enumerated sensor chips
  std::vector<SensorChipPtr> sensor_chips_;
};

}  // namespace system_monitor
}  // namespace cargo