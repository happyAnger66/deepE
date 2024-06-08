#pragma once
#include <boost/chrono.hpp>

#include <map>
#include <string>
#include <vector>
#include "monitor/monitor_item.h"
#include "utils/proc_file.h"

namespace cargo {
namespace system_monitor {

class NetMonitor : public MonitorItem {
  struct netdev_info {
    std::string name;
    int64_t rcv_bytes;
    int64_t rcv_pkts;
    int64_t err_in;
    int64_t drop_in;
    int64_t snd_bytes;
    int64_t snd_pkts;
    int64_t err_out;
    int64_t drop_out;
    int64_t fifo_in;
    int64_t fifo_out;
    boost::chrono::steady_clock::time_point timepoint;
  };

  struct tcp_snmp {
    int64_t in;
    int64_t out;
    int64_t retrans;
    int64_t in_errs;
    int64_t out_rsts;
    int64_t timepoint;
  };

  struct ip_snmp {
    int64_t in_hdr_errs;
    int64_t in_addr_errs;
    int64_t in_unkown_protos;
    int64_t in_discards;
    int64_t out_discards;
    int64_t out_no_routes;
    int64_t reasm_timeout;
    int64_t reasm_reqds;
    int64_t reasm_oks;
    int64_t reasm_fails;
    int64_t frag_oks;
    int64_t frag_fails;
    int64_t frag_creates;
    int64_t timepoint;
  };
  struct udp_snmp {
    int64_t in;
    int64_t out;
    int64_t in_errs;
    int64_t rcvbuf_errs;
    int64_t sndbuf_errs;
    int64_t timepoint;
  };

 public:
  explicit NetMonitor(cargo::proto::SystemMonitorConfig &cfg)
      : MonitorItem(cfg) {}
  int32_t Start(uint32_t interval_ms = 3000) { return 0; }
  int32_t RunOnce(cargo::proto::SystemInfo &msg);
  int32_t Stop() { return 0; }
  const std::string Name() const override { return "NetMonitor"; }

 private:
  std::map<std::string, struct netdev_info> net_devices_;
  std::map<std::string, int64_t> tcp_exts_;
  std::map<std::string, int64_t> ip_exts_;
  void do_snmp(cargo::proto::SystemInfo *msg);
  void do_snmp_udp(cargo::proto::SystemInfo *msg,
                   const std::vector<std::string> &names,
                   const std::vector<std::string> &values);
  void do_snmp_tcp(cargo::proto::SystemInfo *msg,
                   const std::vector<std::string> &names,
                   const std::vector<std::string> &values);
  void do_snmp_ip(cargo::proto::SystemInfo *msg,
                  const std::vector<std::string> &names,
                  const std::vector<std::string> &values);
  void do_netstat(cargo::proto::SystemInfo *msg);
  void do_tcp_ext(cargo::proto::SystemInfo *msg,
                  const std::vector<std::string> &names,
                  const std::vector<std::string> &values);
  void do_ip_ext(cargo::proto::SystemInfo *msg,
                 const std::vector<std::string> &names,
                 const std::vector<std::string> &values);
  int32_t do_gstats(const std::string &devname, cargo::proto::NetInfo *net_msg);
  bool tcp_first_ = true;
  bool udp_first_ = true;
  bool ip_first_ = true;
  tcp_snmp old_tcp_snmp_{};
  udp_snmp old_udp_snmp_{};
  ip_snmp old_ip_snmp_{};
  int64_t last_tcp_ext_time_ = -1;
  int64_t last_ip_ext_time_ = -1;
};

}  // namespace system_monitor
}  // namespace cargo
