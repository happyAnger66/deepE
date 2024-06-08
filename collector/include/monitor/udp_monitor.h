#pragma once

#include <mutex>
#include <thread>
#include <unordered_map>
#include <vector>

#include "cargo_monitor_protos/system_info.pb.h"
#include "monitor/monitor_item.h"

namespace cargo {
namespace system_monitor {

class UdpMonitor : public MonitorItem {
  struct udp_cfg {
    int32_t port;
    int32_t tx_limit;
    int32_t rx_limit;
  };

  struct udp_err {
    int64_t timestamp;
    int32_t port;
    int64_t tx_size;
    int64_t rx_size;
  };

 public:
  explicit UdpMonitor(cargo::proto::SystemMonitorConfig &cfg)
      : MonitorItem(cfg) {}
  int32_t Start(uint32_t interval_ms = 3000) override;
  int32_t RunOnce(cargo::proto::SystemInfo &msg) override;
  int32_t Stop() override {
    run_flag_ = false;
    mon_thread_.join();
    return 0;
  }
  const std::string Name() const override { return "UdpMonitor"; }

 private:
  std::unordered_map<int32_t, udp_cfg> udp_cfgs_;
  std::vector<udp_err> udp_errs_;
  std::mutex mux_;
  std::thread mon_thread_;
  bool run_flag_;
  void mon_udp();
};

}  // namespace system_monitor
}  // namespace cargo
