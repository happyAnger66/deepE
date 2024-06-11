#pragma once
#include <unordered_map>
#include "monitor/monitor_item.h"

namespace cargo {
namespace system_monitor {
class StatusMonitor : public MonitorItem {
 public:
  explicit StatusMonitor(cargo::proto::SystemMonitorConfig &cfg)
      : MonitorItem(cfg) {}
  int32_t Start(uint32_t interval_ms = 3000) override;
  int32_t RunOnce(cargo::proto::SystemInfo &msg) override;
  int32_t Stop() override { return 0; }
  const std::string Name() const override { return "StatusMonitor"; }

 private:
  struct threshold_value {
    float critical;
    float high;
  };
  bool find_cfg_by_name(
      const std::unordered_map<std::string, threshold_value> &cfg,
      const std::string &name, threshold_value &val);
  bool gpu_check(cargo::proto::SystemInfo &msg);
  bool gpu_temp_check(const cargo::proto::GpuInfo &one_gpu,
                      cargo::proto::SystemInfo &msg);
  bool gpu_pwr_check(const cargo::proto::GpuInfo &one_gpu,
                     cargo::proto::SystemInfo &msg);
  bool gpu_mem_check(const cargo::proto::GpuInfo &one_gpu,
                     cargo::proto::SystemInfo &msg);
  bool cpu_temp_check(cargo::proto::SystemInfo &msg);
  bool disk_usage_check(cargo::proto::SystemInfo &msg);
  bool system_check(cargo::proto::SystemInfo &msg);
  bool node_check(cargo::proto::SystemInfo &msg);
  bool node_default_cpu_check(const cargo::proto::NodeInfo &node,
                              cargo::proto::SystemInfo &msg);
  bool node_default_mem_check(const cargo::proto::NodeInfo &node,
                              cargo::proto::SystemInfo &msg);
  bool node_cpu_check(const cargo::proto::NodeInfo &node,
                      cargo::proto::SystemInfo &msg,
                      threshold_value &threshold);
  bool node_mem_check(const cargo::proto::NodeInfo &node,
                      cargo::proto::SystemInfo &msg,
                      threshold_value &threshold);
  threshold_value system_cpu_cfg_;
  threshold_value system_mem_cfg_;
  threshold_value default_cpu_cfg_;
  threshold_value default_mem_cfg_;
  threshold_value cpu_temp_;
  threshold_value disk_usage_;
  threshold_value gpu_temp_;
  threshold_value gpu_pwr_;
  threshold_value gpu_mem_;
  threshold_value gpu_usage_;

  std::unordered_map<std::string, threshold_value> nodes_cpu_cfg_;
  std::unordered_map<std::string, threshold_value> nodes_mem_cfg_;
};
}  // namespace system_monitor
}  // namespace cargo
