#include <iomanip>
#include <vector>

#include <absl/strings/str_split.h>
#include <glog/logging.h>

#include "common_protos/status.pb.h"
#include "monitor/node_latency_monitor.h"
#include "utils/ros_utils.h"

namespace cargo {
namespace system_monitor {

static constexpr int KTimeOutInS = 10;
static constexpr int KUsToMs = 1000;
static constexpr int KUsToS = 1000 * 1000;

using namespace shm_stats;

int32_t NodeLatencyMonitor::Start(uint32_t interval_ms) {
  shm_cnts_mgr_ = shm_stats::ShmCntsMgr::Instance();

  (void)interval_ms;
  return 0;
}

int32_t NodeLatencyMonitor::Stop() {
  shm_cnts_mgr_.reset();
  return 0;
}

int32_t NodeLatencyMonitor::add_pub_info(const std::string &node,
                                         const std::string &topic,
                                         const shm_stats::ShmCntRecord &record,
                                         cargo::proto::SystemInfo &msg) {
  auto pub_info = msg.add_node_pub_info();
  pub_info->set_node(node);
  pub_info->set_topic(topic);
  if (record.avg_delta > 0) {
    pub_info->set_hz(1000 / record.avg_delta);
  }
  pub_info->set_min_delta(record.min_delta / KUsToMs);
  pub_info->set_max_delta(record.max_delta / KUsToMs);
  pub_info->set_avg_delta(record.avg_delta);
  pub_info->set_min_delta_ts(record.min_delta_ts);
  pub_info->set_max_delta_ts(record.max_delta_ts);

  pub_info->set_min_proc_delta(record.min_sub_proc_delta / KUsToMs);
  pub_info->set_max_proc_delta(record.max_sub_proc_delta / KUsToMs);
  pub_info->set_min_proc_delta_ts(record.min_sub_proc_ts);
  pub_info->set_max_proc_delta_ts(record.max_sub_proc_ts);
  pub_info->set_avg_proc_delta(record.avg_sub_proc);
  return 0;
}

int32_t NodeLatencyMonitor::add_sub_info(const std::string &node,
                                         const std::string &topic,
                                         const shm_stats::ShmCntRecord &record,
                                         cargo::proto::SystemInfo &msg) {
  auto sub_info = msg.add_node_sub_info();
  sub_info->set_node(node);
  sub_info->set_topic(topic);
  if (record.avg_delta > 0) {
    sub_info->set_hz(1000 / record.avg_delta);
  }
  sub_info->set_min_delta(record.min_delta / KUsToMs);
  sub_info->set_max_delta(record.max_delta / KUsToMs);
  sub_info->set_avg_delta(record.avg_delta);
  sub_info->set_min_delta_ts(record.min_delta_ts);
  sub_info->set_max_delta_ts(record.max_delta_ts);

  sub_info->set_min_proc_delta(record.min_sub_proc_delta / KUsToMs);
  sub_info->set_max_proc_delta(record.max_sub_proc_delta / KUsToMs);
  sub_info->set_min_proc_delta_ts(record.min_sub_proc_ts);
  sub_info->set_max_proc_delta_ts(record.max_sub_proc_ts);
  sub_info->set_avg_proc_delta(record.avg_sub_proc);

  sub_info->set_min_sched_delta(record.min_sub_sched_delta / KUsToMs);
  sub_info->set_max_sched_delta(record.max_sub_sched_delta / KUsToMs);
  sub_info->set_min_sched_delta_ts(record.min_sub_sched_ts);
  sub_info->set_max_sched_delta_ts(record.max_sub_sched_ts);
  sub_info->set_avg_sched_delta(record.avg_sub_sched);

  sub_info->set_min_ipc(record.min_ipc);
  sub_info->set_max_ipc(record.max_ipc);
  sub_info->set_min_ipc_ts(record.min_ipc_ts);
  sub_info->set_max_ipc_ts(record.max_ipc_ts);
  sub_info->set_avg_ipc(record.avg_ipc);

  return 0;
}

int32_t NodeLatencyMonitor::RunOnce(cargo::proto::SystemInfo &msg) {
  TRACE_EVENT("app", "NodeLatencyMonitor::RunOnce");
  ShmCntsMgr::CntsRecordVec vec;
  if (ShmCntsMgr::GetAllCntsRecords(vec) != 0) {
    LOG(INFO) << "No Shm Records found";
    return -1;
  }

  for (auto &shm_name : vec) {
    std::string node;
    std::string topic;
    int type;
    if (shm_cnts_mgr_->GetRecordInfo(shm_name, node, topic, type) != 0) {
      LOG(INFO) << "Record Info: " << shm_name << " failed.";
      continue;
    }

    ShmCntRecord record{0};
    int ret = shm_cnts_mgr_->GetRecord(node, topic, type, record);
    if (ret != 0 || record.delta_times < 1) {
      continue;
    }

    switch (type) {
      case 0:
        add_pub_info(node, topic, record, msg);
        break;
      case 1:
        add_sub_info(node, topic, record, msg);
        break;
      default:
        LOG(INFO) << "unkown record type: " << type;
        break;
    }
  }
  return 0;
}

}  // namespace system_monitor
}  // namespace cargo
