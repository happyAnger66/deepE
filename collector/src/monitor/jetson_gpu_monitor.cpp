#include <iostream>
#include <string>

#include "monitor/jetson_gpu_monitor.h"
#include "utils/jetsonpower.h"
#include "utils/proc_file.h"
#include "utils/proc_stat.h"
#include "utils/process.h"

namespace cargo {
namespace system_monitor {

// /etc/nvpower/libjetsonpower/jetsonpower_t234.conf

constexpr static char kGpuLoadFile[] = "/sys/devices/gpu.0/load";
constexpr static char kGpuCurFreqFile[] =
    "/sys/kernel/debug/clk/gpusysclk/clk_rate";
constexpr static char kEmcCurFreqFile[] = "/sys/kernel/debug/clk/emc/clk_rate";
constexpr static char kEmcCurLoadFile[] =
    "/sys/kernel/actmon_avg_activity/mc_all";
constexpr static char kCpuTempStr[] = "CPU";
constexpr static char kGpuTempStr[] = "GPU";

int32_t JetsonGpuMonitor::Start(uint32_t interval_ms) {
  if (!jetson_lib_->is_loaded) {
    return -1;
  }
  thread_ = std::make_unique<std::thread>([this]() {
    pthread_setname_np(pthread_self(), "JetsonGpuThread");
    while (!stop_) {
      UpdateGpuInfo();
      std::this_thread::sleep_for(std::chrono::seconds(3));
    }
  });

  (void)interval_ms;
  return 0;
}

void JetsonGpuMonitor::UpdateGpuInfo() {
  float gpu_load = jetson_lib_->JetsonPower_igpu_get_load(0);
  float gpu_freq_mhz = jetson_lib_->JetsonPower_igpu_get_cur_freq(0);

  {
    std::lock_guard<std::mutex> scoped_lock(*mutex_);
    gpu_info_.gpu_freq_mhz = gpu_freq_mhz / 1000.0;
    gpu_info_.gpu_usage = gpu_load;
  }

  update_emc(&gpu_info_);
  update_power(&gpu_info_);
  update_profile(&gpu_info_);
  update_sensors(&cpu_temp_, &gpu_info_);
}

void JetsonGpuMonitor::update_sensors(CpuTemp *cpu_temp, GpuInfo *gpu_info) {
  char **names = jetson_lib_->JetsonPower_sensor_get_names();
  if (names == nullptr) {
    return;
  }

  int nums = jetson_lib_->JetsonPower_sensor_get_nums();
  if (nums <= 0) {
    return;
  }
  int i = 0;
  char *name = nullptr;
  std::vector<GpuSensorInfo> sensors_info;
  for (; i < nums; i++) {
    name = names[i];
    int temp = jetson_lib_->JetsonPower_sensor_get_temp(name);
    if (temp > 0) {
      // auto sensor_info = gpu_info.add_sensors_info();
      // sensor_info->set_name(std::string(name));
      // sensor_info->set_temperature(temp / 1000.0);

      float temp = jetson_lib_->JetsonPower_sensor_get_temp(name) / 1000.0;
      float high =
          jetson_lib_->JetsonPower_sensor_get_sw_throt_temp(name) / 1000.0;
      float crit =
          jetson_lib_->JetsonPower_sensor_get_sw_shutdown_temp(name) / 1000.0;

      struct GpuSensorInfo sensor_info;
      sensor_info.name = std::string(name);
      sensor_info.temperature = temp;
      sensor_info.vaild_temp = true;
      sensor_info.exist_cpu_temp = false;
      sensor_info.exist_gpu_temp = false;

      std::lock_guard<std::mutex> scoped_lock(*mutex_);
      gpu_info->update_count = {true, 0};
      std::size_t found = std::string(name).find(kCpuTempStr);
      if (found != std::string::npos) {
        sensor_info.exist_cpu_temp = true;
        cpu_temp->description = name;
        cpu_temp->temperature = temp;
        cpu_temp->high = high;
        cpu_temp->crit = crit;
        // auto cpu_temperature = msg.add_cpu_temp();
        // cpu_temperature->set_description(name);
        // cpu_temperature->set_temperature(temp);
        // cpu_temperature->set_high(high);
        // cpu_temperature->set_crit(crit);
      }

      found = std::string(name).find(kGpuTempStr);
      if (found != std::string::npos) {
        sensor_info.exist_gpu_temp = true;
        gpu_info->temperature = temp;
        gpu_info->temperature_slow = high;
        gpu_info->temperature_down = crit;
        // gpu_info.set_temperature(temp);
        // gpu_info.set_temperature_slow(high);
        // gpu_info.set_temperature_down(crit);
      }
      sensors_info.push_back(sensor_info);
    }
  }

  {
    std::lock_guard<std::mutex> scoped_lock(*mutex_);
    gpu_info->sensors_info = sensors_info;
  }

  return;
}

void JetsonGpuMonitor::update_power(GpuInfo *gpu_info) {
  char **names = jetson_lib_->JetsonPower_rail_get_names();
  if (names == nullptr) {
    return;
  }

  int nums = jetson_lib_->JetsonPower_rail_get_nums();
  if (nums <= 0) {
    return;
  }
  int i = 0;
  char *name = nullptr;
  std::vector<GpuPowerInfo> powers_info;
  for (; i < nums; i++) {
    name = names[i];

    // auto power_info = gpu_info.add_power_info();
    int in_power = jetson_lib_->JetsonPower_rail_get_power(name);
    int avg_power = jetson_lib_->JetsonPower_rail_get_avg_power(name);
    int warn_power = jetson_lib_->JetsonPower_rail_get_warn_current(name);
    int crit_power = jetson_lib_->JetsonPower_rail_get_crit_current(name);
    // power_info->set_name(std::string(name));
    // power_info->set_inst_power(in_power);
    // power_info->set_avg_power(avg_power);

    struct GpuPowerInfo power_info;
    power_info.name = std::string(name);
    power_info.inst_power = in_power;
    power_info.avg_power = avg_power;
    power_info.exist_gpu_temp = false;
    std::size_t found = std::string(name).find(kGpuTempStr);
    if (found != std::string::npos) {
      power_info.exist_gpu_temp = true;
      // gpu_info.set_pwr_usage(in_power);
      // gpu_info.set_pwr_cap(crit_power);
      std::lock_guard<std::mutex> scoped_lock(*mutex_);
      gpu_info->pwr_usage = in_power;
      gpu_info->pwr_cap = crit_power;
    }
    powers_info.push_back(power_info);
  }

  {
    std::lock_guard<std::mutex> scoped_lock(*mutex_);
    gpu_info->power_info = powers_info;
  }

  return;
}

void JetsonGpuMonitor::update_profile(GpuInfo *gpu_info) {
  char **names = jetson_lib_->JetsonPower_fan_get_names();
  if (names == nullptr) {
    return;
  }

  int nums = jetson_lib_->JetsonPower_fan_get_nums();
  if (nums <= 0) {
    return;
  }
  int i = 0;
  char *name = nullptr;
  std::vector<GpuFanInfo> fans_info;
  for (; i < nums; i++) {
    name = names[i];
    // auto fan_info = gpu_info.add_fan_info();
    int pwm = jetson_lib_->JetsonPower_fan_get_pwm(i + 1);
    int speed = jetson_lib_->JetsonPower_fan_get_speed(i + 1);
    char *profile = jetson_lib_->JetsonPower_fan_get_profile(i + 1);
    // fan_info->set_name(std::string(name));
    // fan_info->set_profile(std::string(profile));
    // fan_info->set_pwm(pwm * 100.0 / 255);
    // fan_info->set_rpm(speed);
    // memset(&gpu_info.fan_info[i], 0, sizeof(struct GpuFanInfo));
    struct GpuFanInfo fan_info;
    fan_info.name = std::string(name);
    fan_info.profile = std::string(profile);
    fan_info.pwm = pwm * 100.0 / 255;
    fan_info.rpm = speed;
    fans_info.push_back(fan_info);
  }

  {
    std::lock_guard<std::mutex> scoped_lock(*mutex_);
    gpu_info->fan_info = fans_info;
  }

  return;
}

void JetsonGpuMonitor::update_emc(GpuInfo *gpu_info) {
  // auto emc_info = gpu_info.mutable_emc_info();
  float emc_load = jetson_lib_->JetsonPower_emc_get_load();
  float emc_freq_mhz = jetson_lib_->JetsonPower_emc_get_cur_freq();
  //  emc_info->set_emc_freq_mhz(emc_freq_mhz / 1000.0);
  // emc_info->set_emc_load(emc_load);

  float total_ram = jetson_lib_->JetsonPower_emc_get_mem_size();
  float free_ram = jetson_lib_->JetsonPower_emc_get_mem_free_size();
  float buffers_ram = jetson_lib_->JetsonPower_emc_get_buffers_size();
  float cached_ram = jetson_lib_->JetsonPower_emc_get_cached_size();
  float total_swap = jetson_lib_->JetsonPower_emc_get_swap_size();
  float free_swap = jetson_lib_->JetsonPower_emc_get_swap_free_size();

  std::lock_guard<std::mutex> scoped_lock(*mutex_);
  // memset(&gpu_info.emc_info, 0, sizeof(struct GpuEmcInfo));
  gpu_info->emc_info.emc_freq_mhz = emc_freq_mhz / 1000.0;
  gpu_info->emc_info.emc_load = emc_load;
  if (total_ram > 0) {
    float mem_used = total_ram - free_ram - buffers_ram - cached_ram;
    float mem_load = mem_used / total_ram * 100.0;
    // emc_info->set_emc_mem_total(total_ram);
    // emc_info->set_emc_mem_used(mem_used);
    // emc_info->set_emc_mem_load(mem_load);

    // gpu_info.set_mem_used_percent(mem_load);
    // gpu_info.set_mem_total(total_ram);

    gpu_info->emc_info.emc_mem_total = total_ram;
    gpu_info->emc_info.emc_mem_used = mem_used;
    gpu_info->emc_info.emc_mem_load = mem_load;

    gpu_info->mem_used_percent = mem_load;
    gpu_info->mem_total = total_ram;
  }

  if (total_swap > 0) {
    int swap_used = total_swap - free_swap;
    float swap_load = swap_used / total_swap * 100.0;
    // emc_info->set_emc_mem_swap_total(total_swap);
    // emc_info->set_emc_mem_swap_used(swap_used);
    // emc_info->set_emc_mem_swap_load(swap_load);

    gpu_info->emc_info.emc_mem_swap_total = total_swap;
    gpu_info->emc_info.emc_mem_swap_used = swap_used;
    gpu_info->emc_info.emc_mem_swap_load = swap_load;
  }
}

void JetsonGpuMonitor::insert_gpu_info(cargo::proto::SystemInfo *msg) {
  cargo::proto::GpuInfo *one_gpu_info = msg->add_gpu_info();
  auto emc_info = one_gpu_info->mutable_emc_info();

  std::lock_guard<std::mutex> scoped_lock(*mutex_);
  one_gpu_info->set_gpu_freq_mhz(gpu_info_.gpu_freq_mhz);
  one_gpu_info->set_gpu_usage(gpu_info_.gpu_usage);
  one_gpu_info->set_mem_used_percent(gpu_info_.mem_used_percent);
  one_gpu_info->set_mem_total(gpu_info_.mem_total);

  // emc info
  emc_info->set_emc_freq_mhz(gpu_info_.emc_info.emc_freq_mhz);
  emc_info->set_emc_load(gpu_info_.emc_info.emc_load);
  emc_info->set_emc_mem_total(gpu_info_.emc_info.emc_mem_total);
  emc_info->set_emc_mem_used(gpu_info_.emc_info.emc_mem_used);
  emc_info->set_emc_mem_load(gpu_info_.emc_info.emc_mem_load);
  emc_info->set_emc_mem_swap_total(gpu_info_.emc_info.emc_mem_swap_total);
  emc_info->set_emc_mem_swap_used(gpu_info_.emc_info.emc_mem_swap_used);
  emc_info->set_emc_mem_swap_load(gpu_info_.emc_info.emc_mem_swap_load);

  // fan info
  for (int i = 0; i < gpu_info_.fan_info.size(); i++) {
    auto fan_info = one_gpu_info->add_fan_info();
    fan_info->set_name(gpu_info_.fan_info[i].name);
    fan_info->set_profile(gpu_info_.fan_info[i].profile);
    fan_info->set_pwm(gpu_info_.fan_info[i].pwm);
    fan_info->set_rpm(gpu_info_.fan_info[i].rpm);
  }

  // power info
  for (int i = 0; i < gpu_info_.power_info.size(); i++) {
    auto power_info = one_gpu_info->add_power_info();
    power_info->set_name(gpu_info_.power_info[i].name);
    power_info->set_inst_power(gpu_info_.power_info[i].inst_power);
    power_info->set_avg_power(gpu_info_.power_info[i].avg_power);
    if (gpu_info_.power_info[i].exist_gpu_temp) {
      one_gpu_info->set_pwr_usage(gpu_info_.pwr_usage);
      one_gpu_info->set_pwr_cap(gpu_info_.pwr_cap);
    }
  }

  // sensor info, cpu temperature, gpu temperature
  for (int i = 0; i < gpu_info_.sensors_info.size(); i++) {
    if (gpu_info_.sensors_info[i].vaild_temp) {
      auto sensor_info = one_gpu_info->add_sensors_info();
      sensor_info->set_name(gpu_info_.sensors_info[i].name);
      sensor_info->set_temperature(gpu_info_.sensors_info[i].temperature);

      if (gpu_info_.sensors_info[i].exist_cpu_temp) {
        auto cpu_temperature = msg->add_cpu_temp();
        cpu_temperature->set_description(cpu_temp_.description);
        cpu_temperature->set_temperature(cpu_temp_.temperature);
        cpu_temperature->set_high(cpu_temp_.high);
        cpu_temperature->set_crit(cpu_temp_.crit);
      }

      if (gpu_info_.sensors_info[i].exist_gpu_temp) {
        one_gpu_info->set_temperature(gpu_info_.temperature);
        one_gpu_info->set_temperature_slow(gpu_info_.temperature_slow);
        one_gpu_info->set_temperature_down(gpu_info_.temperature_down);
      }
    }
  }
}

int32_t JetsonGpuMonitor::RunOnce(cargo::proto::SystemInfo &msg) {
  TRACE_EVENT("app", "JetsonGpuMonitor::RunOnce");
  ProcFile gpu_load_file(kGpuLoadFile);
  ProcFile gpu_cur_freq_file(kGpuCurFreqFile);

  // if the update failed more than 10 times, print an error message
  {
    std::lock_guard<std::mutex> scoped_lock(*mutex_);
    if (!gpu_info_.update_count.first) {
      gpu_info_.update_count.second += 1;
      if (gpu_info_.update_count.second > 10) {
        gpu_info_.update_count.second = 0;
        LOG_EVERY_N(ERROR, 100) << "failed to update gpu info!!!";
      }
    }
    gpu_info_.update_count.first = false;
  }

  // insert data
  insert_gpu_info(&msg);

  // // clear data
  // memset(&gpu_info_, 0, sizeof(struct GpuInfo));
  // memset(&cpu_temp_, 0, sizeof(struct CpuTemp));

  // float gpu_load = jetson_lib_->JetsonPower_igpu_get_load(0);
  // float gpu_freq_mhz = jetson_lib_->JetsonPower_igpu_get_cur_freq(0);

  // one_gpu_info->set_gpu_freq_mhz(gpu_freq_mhz / 1000.0);
  // one_gpu_info->set_gpu_usage(gpu_load);

  // update_emc(*one_gpu_info);
  // update_power(*one_gpu_info);
  // update_profile(*one_gpu_info);
  // update_sensors(msg, *one_gpu_info);
  return 0;
}

}  // namespace system_monitor
}  // namespace cargo
