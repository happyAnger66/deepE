#include <fcntl.h>
#include <string>
#include <unordered_map>

#include <base/base_dir.h>
#include <base/now.h>
#include <common/status.h>
#include <glog/logging.h>
#include <module_protos/module_config.pb.h>

#include "bywire_protos/ChassisInfoRxProto.pb.h"
#include "cargo_monitor_protos/system_monitor_config.pb.h"

#include "av_comm/onboard_config.h"
#include "monitor/monitor_factory.h"
#include "system_monitor_node.h"
#include "tracing/tracer.h"
#include "utils/ros_utils.h"
#include "utils/utils.h"

namespace cargo {

namespace system_monitor {
constexpr char kSystemMonitorMasterTopic[] = "/system_info";
constexpr char kSystemMonitorSlaveTopic[] = "/system_info_slave";
constexpr char kSystemMonitorEnvHz[] = "cargo_monitor_hz";
constexpr char kVehicleModeFile[] = "/dev/shm/vehicle_mode.txt";
constexpr int kMonitorSlaveInterval = 60000;  // 60s

uint32_t SystemMonitorNode::get_timer_interval(uint32_t cfg_interval) {
  char *env_hz = getenv(kSystemMonitorEnvHz);
  int interval = 3000;
  if (env_hz == nullptr) {
    interval = cfg_interval;
  } else {
    int i_interval = atoi(env_hz);
    if (i_interval <= 0) {
      interval = cfg_interval;
    } else {
      interval = 1000 / i_interval;
    }
  }

  LOG(INFO) << "use timer interval:" << interval << "ms";
  return interval;
}

bool SystemMonitorNode::Init() {
  RegisterSub<alpas::proto::BywireChassisInfoRx>(
      av_comm::topic::kChassisByWire,
      [this](const alpas::proto::BywireChassisInfoRx &chassis_msg) {
        uint32_t lon_mode = 0;
        if (chassis_msg.gear_box_info().gearbox_work_mode() ==
                1 /*GEARBOX_WORK_MODE_AUTO*/
            && (chassis_msg.engine_info().eng_work_mode() == 1 ||
                chassis_msg.engine_info().eng_work_mode() == 2)) {
          lon_mode = 1;
        }
        uint32_t lat_mode = 0;
        if (chassis_msg.gear_box_info().gearbox_work_mode() ==
                1 /*GEARBOX_WORK_MODE_AUTO*/
            && (chassis_msg.steer_info().steering_work_mode() == 1 ||
                chassis_msg.steer_info().steering_work_mode() == 2)) {
          lat_mode = 1;
        }
        UpdateMode(lon_mode, lat_mode, chassis_msg.header().timestamp_ns());
      });

  RegisterSub<alpas::proto::PlatoonVehicleInfo>(
      "/alpas/platoon_manager/vehicle_status",
      std::bind(&SystemMonitorNode::TakeOverTimeStamps, this,
                std::placeholders::_1));

  // load node config
  cargo::proto::SystemMonitorConfig config;
  std::string mon_cfg_file;
  if (!RosUtils::GetMonitorConfigFile(mon_cfg_file)) {
    LOG(ERROR) << "cargo_monitor get mon_cfg_file failed !!!";
    return false;
  }

  config =
      av_comm::CreateConfig<cargo::proto::SystemMonitorConfig>(mon_cfg_file);
  LOG(INFO) << "load config  "
            << "\n"
            << config.DebugString();

  slave_node_name_ = config.mon_cfg().slave_node_name();
  slave_host_name_ = config.mon_cfg().slave_host_name();
  slave_cmd_ = config.mon_cfg().slave_cmd();
  for (const auto &monitor_item : config.mon_cfg().monitor_item()) {
    all_monitor_items_.emplace_back(
        MonitorItemFactory::Create(monitor_item, config));
  }

  if (!RosUtils::GetTopicName(topic_name_)) {
    LOG(ERROR) << "cargo_monitor get topic name failed !!!";
    return false;
  }

  auto time_interval = get_timer_interval(config.mon_cfg().interval());
  for (auto it = all_monitor_items_.begin(); it != all_monitor_items_.end();) {
    LOG(INFO) << "item " << (*it)->Name().c_str() << " start begin !!!";
    if (!(*it)->Start(time_interval)) {
      LOG(WARNING) << "item " << (*it)->Name().c_str() << " start success !!!";
      it++;
    } else {
      LOG(ERROR) << "item " << (*it)->Name().c_str() << " start failed !!!";
      it = all_monitor_items_.erase(it);
    }
  }

  // for (auto &monitor_item : all_monitor_items_) {
  //   LOG(INFO) << "item " << monitor_item->Name().c_str() << " start begin
  //   !!!"; if (!monitor_item->Start()) {
  //     LOG(WARNING) << "item " << monitor_item->Name().c_str()
  //                  << " start success !!!";
  //   } else {
  //     LOG(INFO) << "item " << monitor_item->Name().c_str()
  //               << " start failed !!!";

  //   }
  // }

  RegisterPub<cargo::proto::SystemInfo>(topic_name_);

  std::string pub_msg_filename =
      cargo::system_monitor::utils::GetFileInTripPath(name());
  pub_msg_file_ =
      std::ofstream(pub_msg_filename, std::ios::binary | std::ios::trunc);

  // register all timer
  CreateTimer(time_interval, std::bind(&SystemMonitorNode::OnTimer, this));

  /*if (role_ == RosUtils::RosRole::Master && !slave_cmd_.empty()) {
    CreateTimer(kMonitorSlaveInterval,
                std::bind(&SystemMonitorNode::MonitorSlave, this));
  }*/

  return true;
}

SystemMonitorNode::~SystemMonitorNode() {
  for (auto &monitor_item : all_monitor_items_) {
    LOG(INFO) << "item " << monitor_item->Name().c_str() << " stop begin !!!";
    if (!monitor_item->Stop()) {
      LOG(WARNING) << "item " << monitor_item->Name().c_str()
                   << " stop success !!!";
    } else {
      LOG(INFO) << "item " << monitor_item->Name().c_str()
                << " stop failed !!!";
    }
  }
}

void SystemMonitorNode::MonitorSlave() {
  if (-1 != RosUtils::GetNodePid(slave_node_name_)) {
    return;
  }

  std::string cmd = absl::StrFormat(
      "ssh slave docker exec -u root %s /cargo/scripts/run_cmd.sh \"%s\" &",
      slave_host_name_, slave_cmd_);
  (void)system(cmd.c_str());
}

void SystemMonitorNode::write_pb_msg(const cargo::proto::SystemInfo &msg) {
  TRACE_EVENT("app", "pb_msg_write");

  int32_t msg_size = msg.ByteSize();
  pub_msg_file_.write(reinterpret_cast<char *>(&msg_size), sizeof(msg_size));

  std::string m_data;
  msg.SerializeToString(&m_data);
  pub_msg_file_ << m_data;
}

void SystemMonitorNode::OnTimer() {
  TRACE_EVENT("app", "OnTimer");
  cargo::proto::SystemInfo system_info_msg;
  for (const auto &monitor_item : all_monitor_items_)
    monitor_item->RunOnce(system_info_msg);

  auto vehicle_mode_ns = system_info_msg.mutable_vehicle_mode();
  vehicle_mode_ns->set_takeover_ns(takeover_ns_);
  vehicle_mode_ns->set_timestamp_ns(timestamps_ns_);
  vehicle_mode_ns->set_vehicle_event(vehicle_event_);

  Publish(topic_name_, &system_info_msg);
  write_pb_msg(system_info_msg);
}

void SystemMonitorNode::UpdateMode(uint32_t lon_mode, uint32_t lat_mode,
                                   int64_t timestamps_ns) {
  voy::VehicleMode vehicle_mode;
  if (lon_mode == 0) {
    if (lat_mode == 0) {
      vehicle_mode = voy::VehicleMode::MANUAL;
    } else {
      vehicle_mode = voy::VehicleMode::AUTO_STEER;
    }
  } else {
    if (lat_mode == 0) {
      vehicle_mode = voy::VehicleMode::AUTO_SPEED;
    } else {
      vehicle_mode = voy::VehicleMode::AUTO_FULL;
    }
  }

  av_comm::CanbusEngagementTracker::TransitionEvent engage_event =
      canbus_engagement_tracker_.OnMode(vehicle_mode);

  if (engage_event ==
      av_comm::CanbusEngagementTracker::TransitionEvent::kEngagement) {
    TriggerEvent(timestamps_ns, true);
  } else if (engage_event == av_comm::CanbusEngagementTracker::TransitionEvent::
                                 kDisengagement) {
    TriggerEvent(timestamps_ns, false);
  }
}

void SystemMonitorNode::WriteModeToFile() {
  int fd = open(kVehicleModeFile, O_WRONLY | O_CREAT | O_TRUNC, 0644);
  if (fd < 0) {
    LOG(ERROR) << "open vehicle_mode.txt failed: " << strerror(errno);
    return;
  }

  // lock
  if (lockf(fd, F_TLOCK, 0) < 0) {
    LOG(ERROR) << "failed to lock vehicle_mode.txt";
    return;
  }

  // write
  std::string str =
      vehicle_mode_timestamps_ns_ + "\n" + takeover_timestamps_ns_;
  char *buff = const_cast<char *>(str.c_str());
  write(fd, buff, strlen(buff));

  // unlock
  close(fd);
}

void SystemMonitorNode::TakeOverTimeStamps(
    const alpas::proto::PlatoonVehicleInfo &msg) {
  if (last_vehicle_state_msg_.takeover() != msg.takeover() &&
      msg.takeover() == 1) {
    takeover_ns_ = msg.header().timestamp_ns();
    takeover_timestamps_ns_ = std::to_string(msg.header().timestamp_ns());
    WriteModeToFile();
  }
  last_vehicle_state_msg_ = msg;
}

void SystemMonitorNode::TriggerEvent(int64_t &timestamps_ns, bool auto_driver) {
  if (auto_driver) {
    vehicle_event_ = cargo::proto::VehicleEvent::ENGAGEMENT;
  } else {
    vehicle_event_ = cargo::proto::VehicleEvent::DISENGAGEMENT;
  }
  timestamps_ns_ = timestamps_ns;
  vehicle_mode_timestamps_ns_ =
      std::to_string(timestamps_ns) + ":" + std::to_string(int(vehicle_event_));
  WriteModeToFile();
}

}  // namespace system_monitor
}  // namespace cargo
