#pragma once

#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "platoon_protos/PlatoonInfo.pb.h"
#include "voy_protos/canbus.pb.h"

#include "av_comm/topics.h"
#include "module/module.h"
#include "monitor/monitor_item.h"
#include "types/canbus_mode.h"
#include "utils/ros_utils.h"

namespace cargo {
namespace system_monitor {

class SystemMonitorNode : public Module {
 public:
  explicit SystemMonitorNode(const std::string &config_file)
      : Module(config_file){};
  ~SystemMonitorNode();

  virtual bool Init() override;

 private:
  std::vector<std::unique_ptr<MonitorItem> > all_monitor_items_;
  RosUtils::RosRole role_;
  std::string topic_name_;
  std::string slave_node_name_;
  std::string slave_host_name_;
  std::string slave_cmd_;
  std::string takeover_timestamps_ns_;
  std::string vehicle_mode_timestamps_ns_;
  int64_t timestamps_ns_ = 0;
  int64_t takeover_ns_ = 0;
  av_comm::CanbusEngagementTracker canbus_engagement_tracker_;
  alpas::proto::PlatoonVehicleInfo last_vehicle_state_msg_;
  cargo::proto::VehicleEvent vehicle_event_{cargo::proto::VehicleEvent::UNKOWN};

  std::ofstream pub_msg_file_;
  // cargo::proto::VehicleMode* vehicle_mode_ns_;

  void write_pb_msg(const cargo::proto::SystemInfo &msg);
  void OnTimer();
  void MonitorSlave();
  //   void ReceiveCanbusStateCallback(
  //       const cargo::chassis::ChassisInfoRx10ms &canbus_msg);
  void UpdateMode(uint32_t lon_mode, uint32_t lat_mode, int64_t timestamps_ns);
  void WriteModeToFile();
  void TakeOverTimeStamps(const alpas::proto::PlatoonVehicleInfo &msg);
  void TriggerEvent(int64_t &timestamps_ns, bool auto_driver);

  uint32_t get_timer_interval(uint32_t cfg_interval);
  DISALLOW_COPY_AND_ASSIGN(SystemMonitorNode);
};

}  // namespace system_monitor
}  // namespace cargo
