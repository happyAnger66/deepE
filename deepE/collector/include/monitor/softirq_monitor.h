#pragma once
#include <boost/chrono.hpp>

#include <map>
#include <string>
#include <utility>

#include "monitor/monitor_item.h"
#include "utils/proc_file.h"

namespace cargo {
namespace system_monitor {
class SoftIrqMonitor : public MonitorItem {
  struct SoftIrq {
    std::string cpu_name;
    int64_t hi;
    int64_t timer;
    int64_t net_tx;
    int64_t net_rx;
    int64_t block;
    int64_t irq_poll;
    int64_t tasklet;
    int64_t sched;
    int64_t hrtimer;
    int64_t rcu;
    boost::chrono::steady_clock::time_point timepoint;
  };

  struct Irq {
    int64_t nums;
    int64_t ts;
  };

 public:
  explicit SoftIrqMonitor(cargo::proto::SystemMonitorConfig &cfg)
      : MonitorItem(cfg) {}

  int32_t Start(uint32_t interval_ms = 3000) override { return 0; }
  int32_t RunOnce(cargo::proto::SystemInfo &msg) override;
  int32_t Stop() override { return 0; }
  const std::string Name() const override { return "SoftIrqMonitor"; }

 private:
  std::map<std::string, struct SoftIrq> cpu_softirqs_;
  using CpuIrqKey = std::pair<std::string, std::string>;
  std::map<CpuIrqKey, Irq> cpu_irq_infos_;
  void do_irqs(cargo::proto::SystemInfo *msg);
};

}  // namespace system_monitor
}  // namespace cargo
