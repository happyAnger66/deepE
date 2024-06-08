#include <chrono>
#include <string>

#include <boost/algorithm/string.hpp>

#include <base/now.h>
#include "monitor/udp_monitor.h"

#include "utils/proc_file.h"

using namespace std::chrono_literals;

namespace cargo {
namespace system_monitor {

static constexpr char kUdpProcFile[] = "/proc/net/udp";

int32_t UdpMonitor::Start(uint32_t interval_ms) {
  for (auto udp : cfg_.udp_cfg()) {
    udp_cfg u_cfg{0};
    u_cfg.port = udp.port();
    u_cfg.tx_limit = udp.tx_size_limit();
    u_cfg.rx_limit = udp.rx_size_limit();
    if (u_cfg.tx_limit > 0 || u_cfg.rx_limit > 0) {
      udp_cfgs_[u_cfg.port] = u_cfg;
    }
  }

  if (udp_cfgs_.empty()) {
    return 0;
  }

  mon_thread_ = std::thread(&UdpMonitor::mon_udp, this);
  run_flag_ = true;

  (void)interval_ms;
  return 0;
}

void UdpMonitor::mon_udp() {
  while (run_flag_) {
    ProcFile udp_proc(kUdpProcFile);
    std::vector<std::string> segs;
    while (udp_proc.ReadOneLine(segs)) {
      std::string la_str = segs[1];
      std::string tx_rx_str = segs[4];
      std::vector<std::string> ip_port;
      boost::split(ip_port, la_str, boost::is_any_of(":"));

      std::vector<std::string> tx_rx_size;
      boost::split(tx_rx_size, tx_rx_str, boost::is_any_of(":"));

      if (ip_port.size() != 2 || tx_rx_size.size() != 2) {
        segs.clear();
        continue;
      }

      int32_t port = std::stoi(ip_port[1], nullptr, 16);
      int64_t tx_size = std::stoll(tx_rx_size[0], nullptr, 16);
      int64_t rx_size = std::stoll(tx_rx_size[1], nullptr, 16);

      auto iter = udp_cfgs_.find(port);
      if (iter == udp_cfgs_.end()) {
        segs.clear();
        continue;
      }

      auto udp_cfg = iter->second;
      udp_err u_err{0};
      if (tx_size >= udp_cfg.tx_limit || rx_size >= udp_cfg.rx_limit) {
        u_err.timestamp = base::NowNs();
        u_err.port = port;
        u_err.tx_size = tx_size;
        u_err.rx_size = rx_size;
        {
          std::lock_guard<std::mutex> guard(mux_);
          udp_errs_.push_back(u_err);
        }
      }
      segs.clear();
    }
    std::this_thread::sleep_for(100ms);
  }
}

int32_t UdpMonitor::RunOnce(cargo::proto::SystemInfo &msg) {
  TRACE_EVENT("app", "UdpMonitor::RunOnce");
  std::lock_guard<std::mutex> guard(mux_);
  for (auto udp_err : udp_errs_) {
    auto one_udp_msg = msg.add_udp_err();
    one_udp_msg->set_timestamp(udp_err.timestamp);
    one_udp_msg->set_port(udp_err.port);
    one_udp_msg->set_tx_size_64(udp_err.tx_size);
    one_udp_msg->set_rx_size_64(udp_err.rx_size);
  }
  udp_errs_.clear();

  return 0;
}

}  // namespace system_monitor
}  // namespace cargo
