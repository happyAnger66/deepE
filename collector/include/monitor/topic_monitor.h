#pragma once
#include <unordered_map>
#include "cargo_monitor/hz.h"
#include "cargo_monitor_protos/system_info.pb.h"
#include "monitor/monitor_item.h"

namespace cargo {
namespace system_monitor {
class TopicMonitor : public MonitorItem {
  struct TopicInfo {
    std::string name;
    double cfg_hz = -1;
    double cur_hz = -1;
    double error = -1;
    double min_delta = -1;
    double max_delta = -1;
    double std_dev = -1;
    int32_t window = 0;
  };

 public:
  explicit TopicMonitor(cargo::proto::SystemMonitorConfig &cfg)
      : MonitorItem(cfg) {}
  int32_t Start(uint32_t interval_ms = 3000) override;
  int32_t RunOnce(cargo::proto::SystemInfo &msg) override;
  int32_t Stop() override { return 0; }
  const std::string Name() const override { return "TopicMonitor"; }

 private:
  std::shared_ptr<cargo::system_monitor::HzStats> hz_stats_;
  std::map<std::string, cargo::proto::TopicConfig> all_monitor_topics_;
};
}  // namespace system_monitor
}  // namespace cargo
