#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <glog/logging.h>
#include "monitor/monitor_item.h"
#include "utils/jetsonpower.h"

namespace cargo {
namespace system_monitor {

class JetsonGpuMonitor : public MonitorItem {
  struct CpuTemp {
    std::string description;
    double temperature;
    double high;
    double crit;
  };

  struct GpuEmcInfo {
    double emc_freq_mhz;
    double emc_load;
    int32_t emc_mem_total;
    int32_t emc_mem_used;
    double emc_mem_load;
    int32_t emc_mem_swap_total;
    int32_t emc_mem_swap_used;
    double emc_mem_swap_load;
  };

  struct GpuPowerInfo {
    std::string name;
    int32_t inst_power;
    int32_t avg_power;
    bool exist_gpu_temp = false;
  };

  struct GpuFanInfo {
    std::string name;
    std::string profile;
    double pwm;
    int32_t rpm;
  };

  struct GpuSensorInfo {
    std::string name;
    double temperature;
    bool vaild_temp = false;
    bool exist_cpu_temp = false;
    bool exist_gpu_temp = false;
  };

  struct GpuInfo {
    std::string name;
    std::pair<bool, int> update_count = {false, 0};
    double temperature;
    double temperature_down;
    double temperature_slow;
    double pwr_usage;
    double pwr_cap;
    double mem_used_percent;
    double mem_total;
    double gpu_usage;
    double gpu_freq_mhz;  // MHZ
    double gpu_load;      // MHZ
    GpuEmcInfo emc_info;
    std::vector<GpuPowerInfo> power_info;
    std::vector<GpuFanInfo> fan_info;
    std::vector<GpuSensorInfo> sensors_info;
  };

 public:
  explicit JetsonGpuMonitor(cargo::proto::SystemMonitorConfig &cfg)
      : MonitorItem(cfg) {
    jetson_lib_ = JetsonPowerLib::Instance();
  }
  int32_t Start(uint32_t interval_ms = 3000) override;
  int32_t RunOnce(cargo::proto::SystemInfo &msg) override;
  int32_t Stop() override {
    stop_ = true;
    int kill_ret = pthread_kill(thread_->native_handle(), SIGKILL);
    if (kill_ret == ESRCH) {
      LOG(ERROR) << "the specified thread does not exist or has been "
                    "terminated !!!\n ";
    } else if (kill_ret == EINVAL) {
      LOG(ERROR) << "the signal is not valid!!!\n";
    }

    // if (thread_->joinable()) {
    //   thread_->join();
    // }
    return 0;
  }
  const std::string Name() const override { return "JetsonGpuMonitor"; }

 private:
  // void update_emc(cargo::proto::GpuInfo &gpu_info);
  // void update_power(cargo::proto::GpuInfo &gpu_info);
  // void update_profile(cargo::proto::GpuInfo &gpu_info);
  // void update_sensors(cargo::proto::SystemInfo &msg,
  //                     cargo::proto::GpuInfo &gpu_info);

  void update_emc(GpuInfo *gpu_info);
  void update_power(GpuInfo *gpu_info);
  void update_profile(GpuInfo *gpu_info);
  void update_sensors(CpuTemp *cpu_temp, GpuInfo *gpu_info);
  void UpdateGpuInfo();
  void insert_gpu_info(cargo::proto::SystemInfo *msg);
  std::shared_ptr<JetsonPowerLib> jetson_lib_;
  std::shared_ptr<std::mutex> mutex_ = std::make_shared<std::mutex>();
  std::unique_ptr<std::thread> thread_;
  GpuInfo gpu_info_;
  CpuTemp cpu_temp_;
  bool stop_{false};
};

}  // namespace system_monitor
}  // namespace cargo
