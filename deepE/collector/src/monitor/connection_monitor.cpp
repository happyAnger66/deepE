#include <glog/logging.h>
#include "base/now.h"
#include "ros/ros.h"

#include "monitor/connection_monitor.h"
#include "shm_probe/shm_probe.h"
#include "shm_transport/shm_transport_config.h"
#include "utils/netlink_helper.h"
#include "utils/proc_stat.h"
#include "utils/ros_utils.h"

#include "task/thread_task.h"

namespace cargo {
namespace system_monitor {

int32_t ConnectionMonitor::Start(uint32_t interval) {
  interval_ms_ = interval;

  thread_ = std::make_unique<std::thread>(
      std::bind(&ConnectionMonitor::collect_connections, this));
  thread_->detach();

  task_ = std::make_shared<TheadTask>(
      std::bind(&ConnectionMonitor::async_task, this));

  return 0;
}

int32_t ConnectionMonitor::Stop() {
  stop_ = true;
  //  task_.reset(nullptr);
  return 0;
}

void ConnectionMonitor::collect_connections() {
  while (!stop_) {
    ros::V_string local_nodes;
    if (!RosUtils::GetAllLocalNodes(local_nodes)) {
      return;
    }

    std::vector<ros_conn_info> cur_conns;
    for (const auto &node : local_nodes) {
      std::vector<RosUtils::RosConnection> node_conns;

      if (node.find("/record") != std::string::npos) {
        continue;
      }

      if (node.find("/rostopic") != std::string::npos) {
        continue;
      }

      if (!RosUtils::GetNodeConnInfo(node, node_conns)) {
        continue;
      }

      for (auto &one_ros_conn : node_conns) {
        if (filter_connection(one_ros_conn)) {
          continue;
        }

        ros_conn_info ci{};
        ci.node = node;
        ci.topic = one_ros_conn.topic_name;
        ci.direction = one_ros_conn.direction;
        ci.dest_uri = one_ros_conn.dest_uri;
        ci.transport_type = one_ros_conn.transport_type;
        ci.local_port = one_ros_conn.local_port;
        ci.peer_host = one_ros_conn.peer_host;
        ci.peer_port = one_ros_conn.peer_port;
        cur_conns.push_back(ci);
      }
    }

    if (!cur_conns.empty()) {
      std::lock_guard<std::mutex> c_guard{lock_};
      ros_conns_ = cur_conns;
    }

    std::this_thread::sleep_for(std::chrono::seconds(10));
  }

  return;
}

bool ConnectionMonitor::filter_connection(const RosUtils::RosConnection &conn) {
  if (conn.topic_name == "/rosout") {
    return true;
  }

  if (conn.transport_type == "INTRAPROCESS") {
    return true;
  }

  return false;
}

void ConnectionMonitor::appendConnInfo(
    int local_port, const std::string &peer_host, int peer_port,
    int64_t cur_time_ns, std::map<conn_key, conn_stats> *all_conns_cur,
    one_conn_msg_info *one_sock_msg) {
  auto netlink = NetLinkTcp::Instance();
  tcpstat ts;
  if (!netlink->GetOneSocket(local_port, peer_host, peer_port, ts)) {
    return;
  }

  conn_stats cs{};
  cs.local_port = local_port;
  cs.peer_host = peer_host;
  cs.peer_port = peer_port;
  one_sock_msg->mss = ts.mss;
  one_sock_msg->rtt = ts.rtt;

  one_sock_msg->snd_kb = ts.bytes_acked / 1024;
  one_sock_msg->rcv_kb = ts.bytes_received / 1024;
  one_sock_msg->cwnd = ts.cwnd;
  one_sock_msg->un_acked = ts.unacked;

  sockstat *ss = &(ts.ss);
  one_sock_msg->rcv_queue = ss->rq;
  one_sock_msg->snd_queue = ss->wq;

  tcpmem *tm = &(ts.meminfo);
  one_sock_msg->snd_buf = tm->snd_buf;
  one_sock_msg->rcv_buf = tm->rcv_buf;
  one_sock_msg->rmem = tm->rcv_mem;
  one_sock_msg->wmem = tm->snd_mem;
  one_sock_msg->drops = tm->drops;

  cs.snd_bytes = ts.bytes_acked;
  cs.rcv_bytes = ts.bytes_received;
  cs.rcv_bytes = ts.bytes_received;
  cs.retrans_bytes = ts.retrans_bytes;
  cs.retrans = ts.retrans;
  cs.retrans_total = ts.retrans_total;
  cs.update_ns = cur_time_ns;

  auto key = std::make_tuple(cs.local_port, cs.peer_host, cs.peer_port);
  (*all_conns_cur)[key] = cs;

  auto old_conn_iter = all_conns_.find(key);
  if (old_conn_iter != all_conns_.end()) {
    auto old_conn = old_conn_iter->second;

    double time_delta =
        static_cast<double>((cs.update_ns - old_conn.update_ns) / 1e9);
    if (time_delta > 0) {
      one_sock_msg->snd_speed =
          static_cast<float>(cs.snd_bytes - old_conn.snd_bytes) / 1024 /
          time_delta;
      one_sock_msg->rcv_speed =
          static_cast<float>(cs.rcv_bytes - old_conn.rcv_bytes) / 1024 /
          time_delta;
      one_sock_msg->retrans_speed =
          static_cast<float>(cs.retrans_bytes - old_conn.retrans_bytes) / 1024 /
          time_delta;
      one_sock_msg->retrans_segs_speed =
          static_cast<float>(cs.retrans - old_conn.retrans) / 1024 / time_delta;
      one_sock_msg->retrans_segs_total_speed =
          static_cast<float>(cs.retrans_total - old_conn.retrans_total) / 1024 /
          time_delta;
    }

    int64_t snd_delta = cs.snd_bytes - old_conn.snd_bytes;
    int64_t retrans_delta = (cs.retrans_bytes - old_conn.retrans_bytes) * 100;
    if (snd_delta > 0) {
      double retrans_rate = static_cast<double>(retrans_delta) / snd_delta;
      one_sock_msg->retrans_rate = retrans_rate;
    }
  }
}

void ConnectionMonitor::collect_udp(cargo::proto::SystemInfo *msg) {
  if (udp_mon_conns_.empty()) {
    return;
  }

  auto netlink = NetLinkTcp::Instance();
  if (0 != netlink->GetAllUdpSockets()) {
    return;
  }

  for (auto local_port : udp_mon_conns_) {
    udp_diag_info udi{};
    if (!netlink->GetOneUdpSocket(local_port, &udi)) {
      continue;
    }

    auto one_sock_msg = msg->add_udp_info();
    one_sock_msg->set_local_port(udi.lport);
    one_sock_msg->set_rcv_buf(udi.rcv_buf);
    one_sock_msg->set_rcv_mem(udi.rcv_mem);
    one_sock_msg->set_drops(udi.drops);
  }
}

void ConnectionMonitor::async_task() {
  bool is_continue = true;
  {
    std::lock_guard<std::mutex> c_guard{lock_};
    if (ros_conns_.empty() && tcp_mon_conns_.empty()) {
      bool is_continue = false;
    }
  }

  if (!is_continue) {
    std::this_thread::sleep_for(std::chrono::milliseconds(interval_ms_));
    return;
  }

  auto netlink = NetLinkTcp::Instance();
  if (0 != netlink->GetAllSockets()) {
    return;
  }

  int64_t cur_time_ns = base::SteadyClockNowNs();
  std::map<conn_key, conn_stats> all_conns_cur;
  std::vector<one_conn_msg_info> all_conn_msgs_cur;

  {
    std::lock_guard<std::mutex> c_guard{lock_};
    for (auto &conn : ros_conns_) {
      one_conn_msg_info one_conn_msg;
      one_conn_msg.node = conn.node;
      one_conn_msg.topic = conn.topic;
      one_conn_msg.direction = conn.direction;
      one_conn_msg.dst_node = conn.dest_uri;
      one_conn_msg.proto = conn.transport_type;
      one_conn_msg.local_addr =
          std::string("localhost:") + std::to_string(conn.local_port);
      one_conn_msg.peer_addr =
          conn.peer_host + std::string(":") + std::to_string(conn.peer_port);

      appendConnInfo(conn.local_port, conn.peer_host, conn.peer_port,
                     cur_time_ns, &all_conns_cur, &one_conn_msg);
      all_conn_msgs_cur.push_back(one_conn_msg);
    }
  }

  for (auto &conn : tcp_mon_conns_) {
    one_conn_msg_info one_conn_msg;
    one_conn_msg.node = "tcp_mon";
    one_conn_msg.local_addr =
        std::string("localhost:") + std::to_string(conn.local_port);
    one_conn_msg.peer_addr =
        conn.peer_host + std::string(":") + std::to_string(conn.peer_port);
    appendConnInfo(conn.local_port, conn.peer_host, conn.peer_port, cur_time_ns,
                   &all_conns_cur, &one_conn_msg);
    all_conn_msgs_cur.push_back(one_conn_msg);
  }

  {
    std::lock_guard<std::mutex> c_guard{all_cm_lock_};
    all_conn_msgs_ = all_conn_msgs_cur;
  }

  all_conns_ = all_conns_cur;
  std::this_thread::sleep_for(std::chrono::milliseconds(interval_ms_));
}

int32_t ConnectionMonitor::RunOnce(cargo::proto::SystemInfo &msg) {
  TRACE_EVENT("app", "ConnectionMonitor::RunOnce");
  std::lock_guard<std::mutex> c_guard{all_cm_lock_};
  for (const auto &one_conn_msg : all_conn_msgs_) {
    auto one_sock_msg = msg.add_conn_info();
    one_sock_msg->set_nodename(one_conn_msg.node);
    one_sock_msg->set_topic(one_conn_msg.topic);
    one_sock_msg->set_direction(one_conn_msg.direction);
    one_sock_msg->set_dest_node(one_conn_msg.dst_node);
    one_sock_msg->set_proto(one_conn_msg.proto);
    one_sock_msg->set_local_addr(one_conn_msg.local_addr);
    one_sock_msg->set_peer_addr(one_conn_msg.peer_addr);
    one_sock_msg->set_mss(one_conn_msg.mss);
    one_sock_msg->set_rtt(one_conn_msg.rtt);

    one_sock_msg->set_snd_kb(one_conn_msg.snd_kb);
    one_sock_msg->set_rcv_kb(one_conn_msg.rcv_kb);
    one_sock_msg->set_cwnd(one_conn_msg.cwnd);
    one_sock_msg->set_un_acked(one_conn_msg.un_acked);
    one_sock_msg->set_rcv_queue(one_conn_msg.rcv_queue);
    one_sock_msg->set_snd_queue(one_conn_msg.snd_queue);
    one_sock_msg->set_snd_buf(one_conn_msg.snd_buf);
    one_sock_msg->set_rcv_buf(one_conn_msg.rcv_buf);
    one_sock_msg->set_rmem(one_conn_msg.rmem);
    one_sock_msg->set_wmem(one_conn_msg.wmem);
    one_sock_msg->set_drops(one_conn_msg.drops);
    one_sock_msg->set_snd_speed(one_conn_msg.snd_speed);
    one_sock_msg->set_rcv_speed(one_conn_msg.rcv_speed);
    one_sock_msg->set_retrans_speed(one_conn_msg.retrans_speed);
    one_sock_msg->set_retrans_segs_speed(one_conn_msg.retrans_segs_speed);
    one_sock_msg->set_retrans_segs_total_speed(
        one_conn_msg.retrans_segs_total_speed);
    one_sock_msg->set_retrans_rate(one_conn_msg.retrans_rate);
  }
  return 0;
}

}  // namespace system_monitor
}  // namespace cargo
