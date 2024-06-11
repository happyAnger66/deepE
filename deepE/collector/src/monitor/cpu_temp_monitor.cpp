#include "monitor/cpu_temp_monitor.h"

#include <regex>

#include <dlfcn.h>
#include <glog/logging.h>

#include "base/util.h"

#include "utils/proc_stat.h"
#include "utils/process.h"

namespace cargo {
namespace system_monitor {

int32_t CpuTempMonitor::Start(uint32_t interval_ms) {
  void *libjetsonpower_handle = dlopen("libjetsonpower.so", RTLD_NOW);
  if (libjetsonpower_handle != nullptr) {
    dlclose(libjetsonpower_handle);
    return -1;
  }

  sensors_cleanup();
  if (sensors_init(NULL) != 0) {
    LOG(ERROR) << "Failed to initialize sensors library";
    return 1;
  }
  sensor_chips_.clear();

  sensors_chip_name const *chip_name;
  int number = 0;
  while ((chip_name = sensors_get_detected_chips(NULL, &number)) != NULL) {
    SensorChipPtr sensor(new SensorChip(chip_name));
    sensor_chips_.push_back(sensor);
  }
  if (sensor_chips_.size() <= 0) {
    LOG(ERROR) << "No sensors detected";
    return -1;
  }

  (void)interval_ms;
  return 0;
}

int32_t CpuTempMonitor::Stop() {
  sensors_cleanup();
  return 0;
}

int32_t CpuTempMonitor::RunOnce(cargo::proto::SystemInfo &msg) {
  TRACE_EVENT("app", "CpuTempMonitor::RunOnce");
  for (int i = 0; i < sensor_chips_.size(); i++) {
    for (int j = 0; j < sensor_chips_[i]->features_.size(); j++) {
      sensor_chips_[i]->features_[j]->buildStatus(msg);
    }
  }
  return 0;
}

}  // namespace system_monitor
}  // namespace cargo
