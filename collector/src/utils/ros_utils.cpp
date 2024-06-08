#include <stdlib.h>

#include <chrono>
#include <set>
#include <thread>

#include <absl/strings/str_split.h>
#include <glog/logging.h>
#include <boost/regex.hpp>

#include <base/base_dir.h>
#include "ros/master.h"
#include "ros/network.h"
#include "utils/ros_utils.h"
#include "xmlrpcpp/XmlRpc.h"

#include "startup/startup.h"

namespace cargo {
namespace system_monitor {

using namespace std::chrono_literals;

bool RosUtils::GetAllNodes(ros::V_string &nodes) {
  return ros::master::getNodes(nodes);
}

bool RosUtils::GetAllLocalNodes(ros::V_string &nodes) {
  ros::V_string all_nodes;
  if (!GetAllNodes(all_nodes)) {
    return false;
  }

  std::string local_ip;
  if (!GetLocalIp(local_ip)) {
    return false;
  }

  for (auto &node : all_nodes) {
    std::string host;
    uint32_t port;

    if (GetNodeHostAndPort(node, host, port)) {
      if (host == "localhost" || host == local_ip) {
        nodes.push_back(node);
      }
    }
  }

  return true;
}

pid_t RosUtils::GetNodePid(const std::string &name) {
  std::string peer_host;
  uint32_t peer_port;

  if (!GetNodeHostAndPort(name, peer_host, peer_port)) {
    return -1;
  }
  XmlRpc::XmlRpcClient c(peer_host.c_str(), peer_port, "/");
  XmlRpc::XmlRpcValue args;
  XmlRpc::XmlRpcValue result;
  args[0] = ros::this_node::getName();
  // args[0] = this_node::getName();
  if (!c.execute("getPid", args, result)) {
    return -1;
  }

  return result[2];
}

bool RosUtils::GetNodeHostAndPort(const std::string &name, std::string &host,
                                  uint32_t &port) {
  XmlRpc::XmlRpcValue req;
  req[0] = ros::this_node::getName();
  req[1] = name;
  XmlRpc::XmlRpcValue resp;
  XmlRpc::XmlRpcValue payload;

  if (ros::master::execute("lookupNode", req, resp, payload, true)) {
    if (ros::network::splitURI(static_cast<std::string>(resp[2]), host, port)) {
      return true;
    } else {
      LOG(ERROR) << "Bad xml-rpc URI trying to inspect node at: [ " << name
                 << " ]";
    }
  }
  return false;
}

bool RosUtils::GetNodeSubTopics(const std::string &node_name,
                                std::set<std::string> &topics) {
  std::string peer_host;
  uint32_t peer_port;

  if (!GetNodeHostAndPort(node_name, peer_host, peer_port)) {
    return false;
  }
  XmlRpc::XmlRpcClient c(peer_host.c_str(), peer_port, "/");
  XmlRpc::XmlRpcValue req2;
  XmlRpc::XmlRpcValue resp2;
  req2[0] = ros::this_node::getName();
  c.execute("getBusInfo", req2, resp2);

  if (!c.isFault() && resp2.valid() && resp2.size() > 0 &&
      static_cast<int>(resp2[0]) == 1) {
    for (int i = 0; i < resp2[2].size(); i++) {
      XmlRpc::XmlRpcValue info = resp2[2][i];

      if (info.size() >= 7) {
        std::string direction = (std::string)info[2];
        std::string topic = (std::string)info[4];

        if (direction == "i") {
          topics.insert(topic);
        }
      } else {
        LOG(INFO) << "Node at: [" << node_name
                  << " ] failed to return subscriptions.";
        return false;
      }
    }

    return true;
  }

  return false;
}

bool RosUtils::GetNodeConnInfo(const std::string &node_name,
                               std::vector<RosConnection> &connections) {
  static boost::regex kTransportRe(
      "\\w+ connection on port (\\d+) to \\[(.*) on socket (\\d+)\\]");
  std::string peer_host;
  uint32_t peer_port;

  if (!GetNodeHostAndPort(node_name, peer_host, peer_port)) {
    return false;
  }
  XmlRpc::XmlRpcClient c(peer_host.c_str(), peer_port, "/");
  XmlRpc::XmlRpcValue req2;
  XmlRpc::XmlRpcValue resp2;
  req2[0] = ros::this_node::getName();
  c.execute("getBusInfo", req2, resp2);

  if (!c.isFault() && resp2.valid() && resp2.size() > 0 &&
      static_cast<int>(resp2[0]) == 1) {
    for (int i = 0; i < resp2[2].size(); i++) {
      XmlRpc::XmlRpcValue info = resp2[2][i];

      if (info.size() >= 7) {
        RosConnection conn{};
        std::string transport_info;
        conn.conn_id = static_cast<int>(info[0]);
        conn.dest_uri = (std::string)info[1];
        conn.direction = (std::string)info[2];
        conn.transport_type = (std::string)info[3];
        conn.topic_name = (std::string)info[4];
        conn.is_connected = static_cast<bool>(info[5]);

        transport_info = (std::string)info[6];
        boost::cmatch what;
        if (boost::regex_match(transport_info.c_str(), what, kTransportRe)) {
          std::string local_port(what[1].first, what[1].second);
          std::string peer(what[2].first, what[2].second);
          std::string sock_fd(what[3].first, what[3].second);

          std::vector<std::string> peer_info = absl::StrSplit(peer, ':');
          conn.local_port = std::stoi(local_port);
          conn.sock_fd = std::stoi(sock_fd);
          conn.peer_host = peer_info[0];
          conn.peer_port = std::stoi(peer_info[1]);
        }

        connections.push_back(conn);
      }
    }
  } else {
    /*LOG(INFO) << "Node at: [" << node_name
              << " ] failed to return subscriptions.";*/
    return false;
  }

  return true;
}

bool RosUtils::GetMasterIp(std::string &master_ip) {
  static boost::regex kMasterUriRe("(\\w+)://([\\w\\.]+):(\\d+)");
  char *master_uri = getenv("ROS_MASTER_URI");
  boost::cmatch what;
  if (boost::regex_match(master_uri, what, kMasterUriRe)) {
    master_ip = std::string(what[2].first, what[2].second);
    return true;
  } else {
    return false;
  }
}

bool RosUtils::GetLocalIp(std::string &ip) {
  char *local_ip = getenv("ROS_IP");
  if (!local_ip) {
    return false;
  }

  ip = std::string(local_ip);
  return true;
}

RosUtils::RosRole RosUtils::GetRole() {
  char *debug_role = getenv("DEBUG_ROLE");
  if (debug_role) {
    std::string role(debug_role);
    if (role == "master") {
      return RosRole::Master;
    } else {
      return RosRole::Slave;
    }
  }

  std::string local_ip;
  if (!GetLocalIp(local_ip)) {
    return RosRole::Master;
  }

  std::string master_ip;
  if (!GetMasterIp(master_ip)) {
    return RosRole::Slave;
  }

  if (master_ip == local_ip || master_ip == "localhost") {
    return RosRole::Master;
  }

  return RosRole::Slave;
}

bool RosUtils::GetTopicName(std::string &topic) {
  std::string devname;
  int ret = -1;
  ret = ::startup::GetCurrentDevName(devname);
  if (ret != 0) {
    return false;
  }

  if (devname == "master" || devname.empty()) {
    topic = "/system_info";
  } else {
    topic = "/system_info_" + devname;
  }
  return true;
}

bool RosUtils::GetNodeConfigFile(std::string &cfg_file) {
  std::string devname;
  int ret = -1;
  ret = ::startup::GetCurrentDevName(devname);

  LOG(INFO) << "devname: " << devname;
  if (ret != 0) {
    return false;
  }

  if (devname.empty()) {
    devname = "master";
  }

  cfg_file = base::GetBaseDirPath(base::BaseDir::kConfigDir) +
             "/config/node/system_monitor/system_monitor_node_" + devname +
             ".pb.conf";
  LOG(INFO) << "cfg_file: " << cfg_file;
  return true;
}

bool RosUtils::GetMonitorConfigFile(std::string &cfg_file) {
  std::string devname;
  int ret = -1;
  ret = ::startup::GetCurrentDevName(devname);
  if (ret != 0) {
    return false;
  }

  if (devname.empty()) {
    devname = "master";
  }

  cfg_file = "system_monitor_config_" + devname;
  return true;
}

}  // namespace system_monitor
}  // namespace cargo
