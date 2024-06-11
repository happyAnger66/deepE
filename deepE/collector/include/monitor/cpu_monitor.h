#pragma once
#include <boost/chrono.hpp>

#include "monitor/monitor_item.h"
#include "utils/proc_stat.h"

namespace cargo {
namespace system_monitor {

class CpuMonitor : public MonitorItem {
 public:
  explicit CpuMonitor(cargo::proto::SystemMonitorConfig &cfg)
      : MonitorItem(cfg) {}
  int32_t Start(uint32_t interval_ms = 3000) override { return 0; }
  int32_t RunOnce(cargo::proto::SystemInfo &msg) override;
  int32_t Stop() override { return 0; }
  const std::string Name() const override { return "CpuMonitor"; }

 private:
  int64_t cpu_busy_time(const ProcStat::CpuStat &stat);
  int64_t cpu_total_time(const ProcStat::CpuStat &stat);
  int32_t collect_all_cpu_stat(cargo::proto::SystemInfo &msg);
  int32_t collect_one_cpu_stat(const ProcStat::CpuStat &cur,
                               const ProcStat::CpuStat &prev,
                               cargo::proto::SystemInfo &msg);
  int32_t collect_load_info(cargo::proto::SystemInfo &msg);
  bool first_ = true;
  bool sys_info_first_ = true;
  ProcStat::CpuStat cpu_stat_last_;
  ProcStat::SysInfo sys_info_prev_;
  std::vector<ProcStat::CpuStat> all_stat_prev_;
  std::vector<ProcStat::CpuStat> all_stat_cur_;
};

}  // namespace system_monitor
}  // namespace cargo