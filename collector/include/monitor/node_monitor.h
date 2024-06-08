#pragma once

#include <map>
#include <string>
#include <unordered_map>

#include "monitor/monitor_item.h"
#include "utils/process.h"

namespace cargo {
namespace system_monitor {
class NodeMonitor : public MonitorItem {
 public:
  explicit NodeMonitor(const cargo::proto::SystemMonitorConfig &cfg)
      : MonitorItem(cfg) {}
  int32_t Start(uint32_t interval_ms = 3000) override { return 0; }
  int32_t RunOnce(cargo::proto::SystemInfo &msg) override;
  int32_t Stop() override { return 0; }
  const std::string Name() const override { return "NodeMonitor"; }

 private:
  bool is_digits(const std::string &str);
  cargo::proto::NodeInfo *AddCpuTime(const std::string &node_name, pid_t pid,
                                     cargo::proto::SystemInfo *msg);
  cargo::proto::NodeInfo *AddMemUsed(const std::string &node_name, pid_t pid,
                                     cargo::proto::SystemInfo *msg,
                                     cargo::proto::NodeInfo *one_node_msg);
  bool AddIoStat(const std::string &node_name, pid_t pid,
                 cargo::proto::NodeInfo *msg);
  bool AddSchedTimes(const std::string &node_name, pid_t pid,
                     cargo::proto::NodeInfo *msg);
  bool AddProcStatus(const std::string &node_name, pid_t pid,
                     cargo::proto::NodeInfo *msg);
  bool AddNodeSockets(const std::string &node_name, pid_t pid,
                      cargo::proto::NodeInfo *msg);
  std::map<int, Process::CpuTime> nodes_cpu_;
  std::map<int, Process::SchedTime> nodes_sched_time_;
  std::map<int, Process::IoCounter> nodes_io_;
};

}  // namespace system_monitor
}  // namespace cargo
