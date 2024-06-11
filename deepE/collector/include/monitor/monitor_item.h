#pragma once

#include <string>
#include <unordered_map>

#include "tracing/tracer.h"

#include "cargo_monitor_protos/system_info.pb.h"
#include "cargo_monitor_protos/system_monitor_config.pb.h"

namespace cargo {
namespace system_monitor {

constexpr int KNsToS = 1000 * 1000 * 1000;

class MonitorItem {
 public:
  explicit MonitorItem(const cargo::proto::SystemMonitorConfig &cfg)
      : cfg_(cfg) {}
  virtual ~MonitorItem() {}
  enum class ItemType {
    ItemCpu,
    ItemCpuTemp,
    ItemGpu,
    ItemGpuJetson,
    ItemMem,
    ItemFs,
    ItemNet,
    ItemNode,
    ItemSocket,
    ItemTopic,
    ItemSubLat,
    ItemStatus,
    ItemSoftIrq,
    ItemUdp,
    ItemNodeLatency,
    ItemThreads,
    ItemMax,
  };

  static ItemType GetItemTypeByName(const std::string &name) {
    static std::unordered_map<std::string, ItemType> item_map = {
        {"cpu", ItemType::ItemCpu},
        {"cpu_temp", ItemType::ItemCpuTemp},
        {"gpu", ItemType::ItemGpu},
        {"gpu_jetson", ItemType::ItemGpuJetson},
        {"mem", ItemType::ItemMem},
        {"fs", ItemType::ItemFs},
        {"net", ItemType::ItemNet},
        {"node", ItemType::ItemNode},
        {"connection", ItemType::ItemSocket},
        {"topic", ItemType::ItemTopic},
        {"sub_latency", ItemType::ItemSubLat},
        {"status", ItemType::ItemStatus},
        {"softirq", ItemType::ItemSoftIrq},
        {"udp", ItemType::ItemUdp},
        {"node_latency", ItemType::ItemNodeLatency},
        {"threads", ItemType::ItemThreads}};

    auto find_iter = item_map.find(name);
    if (find_iter != item_map.end()) {
      return (*find_iter).second;
    }

    return ItemType::ItemMax;
  }

  virtual int32_t Start(uint32_t interval = 3000) = 0;
  virtual int32_t RunOnce(cargo::proto::SystemInfo &system_info_msg) = 0;
  virtual int32_t Stop() = 0;
  virtual const std::string Name() const { return "MonitorItem"; }

 protected:
  cargo::proto::SystemMonitorConfig cfg_;
};

}  // namespace system_monitor
}  // namespace cargo
