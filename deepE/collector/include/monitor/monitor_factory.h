#pragma once

#include <memory>

#include <boost/algorithm/string.hpp>
#include "connection_monitor.h"
#include "cpu_monitor.h"
#include "cpu_temp_monitor.h"
#include "fs_monitor.h"

#include "gpu_monitor.h"
#include "jetson_gpu_monitor.h"

#include "mem_monitor.h"
#include "monitor_item.h"
#include "msglat_monitor.h"
#include "net_monitor.h"
#include "node_latency_monitor.h"
#include "node_monitor.h"
#include "softirq_monitor.h"
#include "status_monitor.h"
#include "thread_monitor.h"
#include "topic_monitor.h"
#include "udp_monitor.h"

namespace cargo {
namespace system_monitor {
class MonitorItemFactory {
 public:
  static std::unique_ptr<MonitorItem> Create(
      const std::string &name, cargo::proto::SystemMonitorConfig &cfg) {
    std::string lower_name(name);
    std::transform(name.begin(), name.end(), lower_name.begin(), ::tolower);
    return CreateMonitorItem(MonitorItem::GetItemTypeByName(lower_name), cfg);
  }

 private:
  static std::unique_ptr<MonitorItem> CreateMonitorItem(
      MonitorItem::ItemType type, cargo::proto::SystemMonitorConfig &cfg) {
    switch (type) {
      case MonitorItem::ItemType::ItemCpu:
        return std::make_unique<CpuMonitor>(CpuMonitor(cfg));
        break;
      case MonitorItem::ItemType::ItemCpuTemp:
        return std::make_unique<CpuTempMonitor>(CpuTempMonitor(cfg));
        break;
      case MonitorItem::ItemType::ItemMem:
        return std::make_unique<MemMonitor>(MemMonitor(cfg));
        break;
      case MonitorItem::ItemType::ItemFs:
        return std::make_unique<FsMonitor>(FsMonitor(cfg));
        break;
      case MonitorItem::ItemType::ItemNet:
        return std::make_unique<NetMonitor>(NetMonitor(cfg));
        break;
      case MonitorItem::ItemType::ItemNode:
        return std::make_unique<NodeMonitor>(NodeMonitor(cfg));
        break;
      case MonitorItem::ItemType::ItemSocket:
        return std::make_unique<ConnectionMonitor>(cfg);
        break;
      case MonitorItem::ItemType::ItemTopic:
        return std::make_unique<TopicMonitor>(TopicMonitor(cfg));
        break;
      case MonitorItem::ItemType::ItemSubLat:
        return std::make_unique<MsgLatMonitor>(MsgLatMonitor(cfg));
        break;
      case MonitorItem::ItemType::ItemStatus:
        return std::make_unique<StatusMonitor>(StatusMonitor(cfg));
        break;
      case MonitorItem::ItemType::ItemGpuJetson:
        return std::make_unique<JetsonGpuMonitor>(JetsonGpuMonitor(cfg));
        break;
      case MonitorItem::ItemType::ItemGpu:
        return std::make_unique<GpuMonitor>(GpuMonitor(cfg));
        break;
      case MonitorItem::ItemType::ItemSoftIrq:
        return std::make_unique<SoftIrqMonitor>(SoftIrqMonitor(cfg));
        break;
      case MonitorItem::ItemType::ItemUdp:
        return std::make_unique<UdpMonitor>(cfg);
        break;
      case MonitorItem::ItemType::ItemNodeLatency:
        return std::make_unique<NodeLatencyMonitor>(cfg);
        break;
      case MonitorItem::ItemType::ItemThreads:
        return std::make_unique<ThreadMonitor>(cfg);
        break;
      default:
        LOG(ERROR) << "unkown monitor item: [ " << static_cast<int>(type)
                   << " ]";
        break;
    }

    return nullptr;
  }
};

}  // namespace system_monitor
}  // namespace cargo
