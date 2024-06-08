#include <dlfcn.h>
#include <glog/logging.h>

#include "monitor/gpu_monitor.h"
#include "utils/proc_stat.h"
#include "utils/process.h"

namespace cargo {
namespace system_monitor {

template <typename Func>
Func GpuMonitor::Load(void *handle, const std::string &name) const {
  void *symbol = dlsym(handle, name.c_str());
  auto *err = dlerror();
  CHECK(err == nullptr) << err;
  return reinterpret_cast<Func>(symbol);
}

int32_t GpuMonitor::Start(uint32_t interval_ms) {
  void *libjetsonpower_handle = dlopen("libjetsonpower.so", RTLD_NOW);
  if (libjetsonpower_handle != nullptr) {
    dlclose(libjetsonpower_handle);
    return -1;
  }
  (void)interval_ms;
  return 0;
}

int32_t GpuMonitor::RunOnce(cargo::proto::SystemInfo &msg) {
  TRACE_EVENT("app", "GpuMonitor::RunOnce");
  nvmlReturn_t ret;

  void *libnvidia = dlopen("libnvidia-ml.so.1", RTLD_LAZY);

  if (libnvidia) {
    ret = Load<decltype(&nvmlInit)>(libnvidia, "nvmlInit")();
    if (NVML_SUCCESS != ret) {
      LOG(ERROR) << "Failed to initialize NVML ("
                 << Load<decltype(&nvmlErrorString)>(libnvidia,
                                                     "nvmlErrorString")(ret)
                 << ")";
      return -1;
    }

    uint32_t device_count;
    ret = Load<decltype(&nvmlDeviceGetCount)>(
        libnvidia, "nvmlDeviceGetCount")(&device_count);
    // ret = nvmlDeviceGetCount(&device_count);
    if (NVML_SUCCESS != ret) {
      LOG(ERROR) << "Failed to query device count ("
                 << Load<decltype(&nvmlErrorString)>(libnvidia,
                                                     "nvmlErrorString")(ret)
                 << ")";
      goto ERROR;
    }

    for (uint32_t i = 0; i < device_count; i++) {
      nvmlDevice_t device;
      ret = Load<decltype(&nvmlDeviceGetHandleByIndex)>(
          libnvidia, "nvmlDeviceGetHandleByIndex")(i, &device);
      // ret = nvmlDeviceGetHandleByIndex(i, &device);
      if (NVML_SUCCESS != ret) {
        LOG(ERROR) << "Failed to get handle for device " << i << "("
                   << Load<decltype(&nvmlErrorString)>(libnvidia,
                                                       "nvmlErrorString")(ret)
                   << ")";
        continue;
      }

      char name[NVML_DEVICE_NAME_BUFFER_SIZE];

      ret = Load<decltype(&nvmlDeviceGetName)>(libnvidia, "nvmlDeviceGetName")(
          device, name, NVML_DEVICE_NAME_BUFFER_SIZE);
      // ret = nvmlDeviceGetName(device, name, NVML_DEVICE_NAME_BUFFER_SIZE);
      if (NVML_SUCCESS != ret) {
        LOG(ERROR) << "Failed to get name of device " << i << "("
                   << Load<decltype(&nvmlErrorString)>(libnvidia,
                                                       "nvmlErrorString")(ret)
                   << ")";
        continue;
      }

      nvmlPciInfo_t pci;
      ret = Load<decltype(&nvmlDeviceGetPciInfo)>(
          libnvidia, "nvmlDeviceGetPciInfo")(device, &pci);
      // ret = nvmlDeviceGetPciInfo(device, &pci);
      if (NVML_SUCCESS != ret) {
        LOG(ERROR) << "Failed to get pci info for device " << i << "("
                   << Load<decltype(&nvmlErrorString)>(libnvidia,
                                                       "nvmlErrorString")(ret)
                   << ")";
        continue;
      }

      uint32_t temperature_shutdown;

      ret = Load<decltype(&nvmlDeviceGetTemperatureThreshold)>(
          libnvidia, "nvmlDeviceGetTemperatureThreshold")(
          device, NVML_TEMPERATURE_THRESHOLD_SHUTDOWN, &temperature_shutdown);
      // ret = nvmlDeviceGetTemperatureThreshold(
      //   device, NVML_TEMPERATURE_THRESHOLD_SHUTDOWN, &temperature_shutdown);
      if (NVML_SUCCESS != ret) {
        LOG(ERROR) << "device " << i
                   << " Failed to get NVML_TEMPERATURE_THRESHOLD_SHUTDOWN: "
                   << Load<decltype(&nvmlErrorString)>(libnvidia,
                                                       "nvmlErrorString")(ret);
        continue;
      }

      uint32_t temperature_slowdown;

      ret = Load<decltype(&nvmlDeviceGetTemperatureThreshold)>(
          libnvidia, "nvmlDeviceGetTemperatureThreshold")(
          device, NVML_TEMPERATURE_THRESHOLD_SLOWDOWN, &temperature_slowdown);
      // ret = nvmlDeviceGetTemperatureThreshold(
      //    device, NVML_TEMPERATURE_THRESHOLD_SLOWDOWN, &temperature_slowdown);
      if (NVML_SUCCESS != ret) {
        LOG(ERROR) << "device " << i
                   << " Failed to get NVML_TEMPERATURE_THRESHOLD_SLOWDOWN: "
                   << Load<decltype(&nvmlErrorString)>(libnvidia,
                                                       "nvmlErrorString")(ret);
        continue;
      }

      uint32_t temperature;
      ret = Load<decltype(&nvmlDeviceGetTemperature)>(
          libnvidia, "nvmlDeviceGetTemperature")(device, NVML_TEMPERATURE_GPU,
                                                 &temperature);
      // ret =
      //    nvmlDeviceGetTemperature(device, NVML_TEMPERATURE_GPU,
      //    &temperature);
      if (NVML_SUCCESS != ret) {
        LOG(ERROR) << "device " << i << " Failed to get NVML_TEMPERATURE_GPU: "
                   << Load<decltype(&nvmlErrorString)>(libnvidia,
                                                       "nvmlErrorString")(ret);
        continue;
      }

      uint32_t power_usage;
      ret = Load<decltype(&nvmlDeviceGetPowerUsage)>(
          libnvidia, "nvmlDeviceGetPowerUsage")(device, &power_usage);
      // ret = nvmlDeviceGetPowerUsage(device, &power_usage);
      if (NVML_SUCCESS != ret) {
        LOG(ERROR) << "device " << i << " Failed to get PowerUsage: "
                   << Load<decltype(&nvmlErrorString)>(libnvidia,
                                                       "nvmlErrorString")(ret);
        continue;
      }

      uint32_t power_limit;
      ret = Load<decltype(&nvmlDeviceGetPowerManagementLimit)>(
          libnvidia, "nvmlDeviceGetPowerManagementLimit")(device, &power_limit);
      // ret = nvmlDeviceGetPowerManagementLimit(device, &power_limit);
      if (NVML_SUCCESS != ret) {
        LOG(ERROR) << "device " << i << " Failed to get PowerManagementLimit: "
                   << Load<decltype(&nvmlErrorString)>(libnvidia,
                                                       "nvmlErrorString")(ret);
        continue;
      }

      nvmlMemory_t memory;
      ret = Load<decltype(&nvmlDeviceGetMemoryInfo)>(
          libnvidia, "nvmlDeviceGetMemoryInfo")(device, &memory);
      // ret = nvmlDeviceGetMemoryInfo(device, &memory);
      if (NVML_SUCCESS != ret) {
        LOG(ERROR) << "device " << i << "Failed to get MemoryInfo: "
                   << Load<decltype(&nvmlErrorString)>(libnvidia,
                                                       "nvmlErrorString")(ret);
        continue;
      }

      nvmlUtilization_t utilization;
      ret = Load<decltype(&nvmlDeviceGetUtilizationRates)>(
          libnvidia, "nvmlDeviceGetUtilizationRates")(device, &utilization);
      // ret = nvmlDeviceGetUtilizationRates(device, &utilization);
      if (NVML_SUCCESS != ret) {
        LOG(ERROR) << "device " << i << "Failed to get Gpu usage: "
                   << Load<decltype(&nvmlErrorString)>(libnvidia,
                                                       "nvmlErrorString")(ret);
        continue;
      }

      auto one_gpu_info = msg.add_gpu_info();
      one_gpu_info->set_name(name);
      one_gpu_info->set_gpu_usage(utilization.gpu);
      one_gpu_info->set_temperature(temperature);
      one_gpu_info->set_temperature_down(temperature_shutdown);
      one_gpu_info->set_temperature_slow(temperature_slowdown);
      one_gpu_info->set_pwr_usage(power_usage * 1.0 / 1000);
      one_gpu_info->set_pwr_cap(power_limit * 1.0 / 1000);
      one_gpu_info->set_mem_used_percent(memory.used * 100.0 / memory.total);
      one_gpu_info->set_mem_total(memory.total * 1.0 / 1024 / 1024);

      GpuProcesses(device, msg);
    }

  ERROR:
    ret = Load<decltype(&nvmlShutdown)>(libnvidia, "nvmlShutdown")();
    // ret = nvmlShutdown();
    if (NVML_SUCCESS != ret) {
      LOG(ERROR) << "Failed to shutdown NVML: "
                 << Load<decltype(&nvmlErrorString)>(libnvidia,
                                                     "nvmlErrorString")(ret);
    }
    return 0;
  } else {
    LOG_FIRST_N(ERROR, 1) << "libnvidia-ml.so not find!";
    return -1;
  }
}

bool GpuMonitor::GpuProcesses(nvmlDevice_t device,
                              cargo::proto::SystemInfo &msg) {
  void *libnvidia = dlopen("libnvidia-ml.so.1", RTLD_LAZY);
  if (libnvidia) {
    unsigned int proc_cnts = 0;
    nvmlProcessInfo_t *infos = nullptr;
    nvmlReturn_t ret = Load<decltype(&nvmlDeviceGetComputeRunningProcesses)>(
        libnvidia, "nvmlDeviceGetComputeRunningProcesses")(device, &proc_cnts,
                                                           infos);
    if (ret == NVML_ERROR_INSUFFICIENT_SIZE) {
      int cnt = proc_cnts * 2 + 5;
      infos = new nvmlProcessInfo_t[cnt];
      ret = Load<decltype(&nvmlDeviceGetComputeRunningProcesses)>(
          libnvidia, "nvmlDeviceGetComputeRunningProcesses")(device, &proc_cnts,
                                                             infos);
    }

    if (NVML_SUCCESS != ret) {
      LOG(ERROR) << "device Failed to get process "
                 << Load<decltype(&nvmlErrorString)>(libnvidia,
                                                     "nvmlErrorString")(ret);
      return false;
    }

    nvmlProcessInfo_t *cur_proc = infos;
    for (int i = 0; i < proc_cnts; i++) {
      auto one_gpu_proc = msg.add_gpu_process();

      one_gpu_proc->set_pid(cur_proc->pid);
      std::string name;
      if (ProcStat::Comm(cur_proc->pid, name)) {
        one_gpu_proc->set_name(name);
      }

      one_gpu_proc->set_mem_used(cur_proc->usedGpuMemory / 1024.0 / 1024.0);
      cur_proc++;
    }

    delete[] infos;
    return true;
  } else {
    LOG_FIRST_N(ERROR, 1) << "libnvidia-ml.so not find!";
    return false;
  }
}

}  // namespace system_monitor
}  // namespace cargo
