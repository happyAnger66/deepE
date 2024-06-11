#pragma once

#include <pthread.h>
#include <map>
#include <memory>
#include <string>

#include <ros/ros.h>
#include <boost/noncopyable.hpp>
#include "cargo_monitor_protos/system_monitor_config.pb.h"

namespace cargo {
namespace system_monitor {
class MonitorConfig ::private boost::noncopyable {
  static MonitorConfig& Instance() {
    pthread_once(&once, &MonitorConfig::init);
    return *monitor_config_instance;
  }

  static void init() { monitor_config_instance = new MonitorConfig(); }
  static pthread_once_t once;
  static MonitorConfig* monitor_config_instance;
  cargo::SystemMonitorConfig monitor_cfg_;
};
}  // namespace system_monitor
}  // namespace cargo