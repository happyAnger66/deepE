#pragma once

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

#include "monitor/monitor_item.h"
#include "utils/ros_utils.h"

#include "task/thread_task.h"

namespace cargo {
namespace system_monitor {
class ConnectionMonitor : public MonitorItem {
 public:
  explicit ConnectionMonitor(const cargo::proto::SystemMonitorConfig &cfg)
      : MonitorItem(cfg) {
    for (auto tcp_mon_conn : cfg.tcp_mon_cfg()) {
      tcp_conn_cfg tc{0};
      tc.local_port = tcp_mon_conn.local_port();
      tc.peer_host = tcp_mon_conn.peer_host();
      tc.peer_port = tcp_mon_conn.peer_port();
      tcp_mon_conns_.push_back(tc);
    }

    for (auto udp_mon_conn : cfg.udp_mon_cfg()) {
      udp_mon_conns_.push_back(udp_mon_conn.local_port());
    }
  }
  int32_t Start(uint32_t interval_ms = 3000) override;
  int32_t RunOnce(cargo::proto::SystemInfo &msg) override;
  int32_t Stop() override;
  const std::string Name() const override { return "ConnectionMonitor"; }

 private:
  struct ros_conn_info {
    std::string node;
    std::string topic;
    std::string direction;
    std::string dest_uri;
    std::string transport_type;
    int local_port;
    std::string peer_host;
    int peer_port;
  };

  struct conn_stats {
    int local_port;
    std::string peer_host;
    int peer_port;
    int64_t snd_bytes;
    int64_t rcv_bytes;
    int64_t retrans_total;
    int64_t retrans;
    int64_t retrans_bytes;
    int64_t update_ns;
  };

  struct tcp_conn_cfg {
    int local_port;
    std::string peer_host;
    int peer_port;
  };

  struct one_conn_msg_info {
    std::string node;
    std::string topic;
    std::string direction;
    std::string dst_node;
    std::string proto;
    std::string status;
    std::string local_addr;
    std::string peer_addr;
    float snd_kb;
    float rcv_kb;
    int32_t rcv_buf;
    int32_t snd_buf;
    int32_t rcv_queue;
    int32_t snd_queue;
    int32_t rmem;
    int32_t wmem;
    int32_t mss;
    float rtt;
    float snd_speed;
    float rcv_speed;
    float retrans_speed;
    float retrans_rate;
    int32_t cwnd;
    int64_t un_acked;
    float retrans_segs_speed;
    float retrans_segs_total_speed;
    int64_t drops;
  };
  bool stop_ = false;
  std::mutex lock_;
  std::vector<ros_conn_info> ros_conns_;
  std::vector<tcp_conn_cfg> tcp_mon_conns_;
  std::vector<int> udp_mon_conns_;

  std::vector<one_conn_msg_info> all_conn_msgs_;
  std::mutex all_cm_lock_;
  std::shared_ptr<TheadTask> task_;
  uint32_t interval_ms_;

  std::unique_ptr<std::thread> thread_;
  using conn_key = std::tuple<int, std::string, int>;
  std::map<conn_key, conn_stats> all_conns_;
  void async_task();
  bool filter_connection(const RosUtils::RosConnection &conn);
  void collect_connections();
  void collect_udp(cargo::proto::SystemInfo *msg);
  void appendConnInfo(int local_port, const std::string &peer_host,
                      int peer_port, int64_t cur_time_ns,
                      std::map<conn_key, conn_stats> *all_conns_cur,
                      one_conn_msg_info *one_sock_msg);
};

}  // namespace system_monitor
}  // namespace cargo
