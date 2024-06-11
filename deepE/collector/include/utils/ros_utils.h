#pragma once

#include <sys/types.h>
#include <unistd.h>

#include <set>
#include <string>

#include "ros/ros.h"

namespace cargo {
namespace system_monitor {

class RosUtils {
 public:
  enum class RosRole { Master, Slave, Unkown };
  struct RosConnection {
    int32_t conn_id;
    std::string dest_uri;
    std::string direction;
    std::string transport_type;
    std::string topic_name;
    bool is_connected;
    int32_t local_port;
    std::string peer_host;
    int32_t peer_port;
    int32_t sock_fd;
  };

  static RosRole GetRole();
  static bool GetLocalIp(std::string &ip);
  static bool GetMasterIp(std::string &master_ip);
  static bool GetAllLocalNodes(ros::V_string &nodes);
  static bool GetAllNodes(ros::V_string &nodes);
  static pid_t GetNodePid(const std::string &name);
  static bool GetNodeConnInfo(const std::string &node_name,
                              std::vector<RosConnection> &connections);
  static bool GetNodeHostAndPort(const std::string &name, std::string &host,
                                 uint32_t &port);
  static bool GetNodeSubTopics(const std::string &node_name,
                               std::set<std::string> &topics);
  static bool GetTopicName(std::string &topic);
  static bool GetNodeConfigFile(std::string &cfg_file);
  static bool GetMonitorConfigFile(std::string &cfg_file);
};
}  // namespace system_monitor
}  // namespace cargo