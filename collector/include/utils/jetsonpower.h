#pragma once

#include <dlfcn.h>
#include <pthread.h>

#include <glog/logging.h>
#include <memory>

namespace cargo {
namespace system_monitor {

class JetsonPowerLib {
 public:
  JetsonPowerLib() {
    libjetsonpower_handle_ = dlopen("libjetsonpower.so", RTLD_NOW);
    if (libjetsonpower_handle_ == nullptr) {
      is_loaded = false;
      LOG(ERROR) << "jetsonpower lib dlopen failed";
    } else {
      libjetson_init_func init_func = (libjetson_init_func)dlsym(
          libjetsonpower_handle_, "init_libjetsonpower");
      if (init_func == nullptr || init_func() != 0) {
        LOG(ERROR) << "jetsonpower lib init failed";
        is_loaded = false;
        return;
      }
      is_loaded = true;
      lib_igpu_get_cur_freq_ = (igpu_get_cur_freq_func)dlsym(
          libjetsonpower_handle_, "igpu_get_cur_freq");
      lib_igpu_get_load_ =
          (igpu_get_load_func)dlsym(libjetsonpower_handle_, "igpu_get_load");
      lib_emc_get_cur_freq_ = (emc_get_cur_freq_func)dlsym(
          libjetsonpower_handle_, "emc_get_cur_freq");
      lib_emc_get_load_ =
          (emc_get_load_func)dlsym(libjetsonpower_handle_, "emc_get_load");
      lib_emc_get_mem_size_ = (emc_get_mem_size_func)dlsym(
          libjetsonpower_handle_, "emc_get_mem_size");
      lib_emc_get_mem_free_size_ = (emc_get_mem_free_size_func)dlsym(
          libjetsonpower_handle_, "emc_get_free_mem_size");
      lib_emc_get_buffers_size_ = (emc_get_buffers_size_func)dlsym(
          libjetsonpower_handle_, "emc_get_buffers_size");
      lib_emc_get_cached_size_ = (emc_get_cached_size_func)dlsym(
          libjetsonpower_handle_, "emc_get_cached_size");
      lib_emc_get_swap_size_ = (emc_get_swap_size_func)dlsym(
          libjetsonpower_handle_, "emc_get_swap_size");
      lib_emc_get_swap_free_size_ = (emc_get_swap_free_size_func)dlsym(
          libjetsonpower_handle_, "emc_get_free_swap_size");

      lib_rail_get_nums_ =
          (rail_get_nums_func)dlsym(libjetsonpower_handle_, "rail_get_nums");
      lib_rail_get_names_ =
          (rail_get_names_func)dlsym(libjetsonpower_handle_, "rail_get_names");
      lib_rail_get_current_ = (rail_get_current_func)dlsym(
          libjetsonpower_handle_, "rail_get_current");
      lib_rail_get_power_ =
          (rail_get_power_func)dlsym(libjetsonpower_handle_, "rail_get_power");
      lib_rail_get_voltage_ = (rail_get_voltage_func)dlsym(
          libjetsonpower_handle_, "rail_get_voltage");
      lib_rail_get_warn_current_ = (rail_get_warn_current_func)dlsym(
          libjetsonpower_handle_, "rail_get_warn_current");
      lib_rail_get_crit_current_ = (rail_get_crit_current_func)dlsym(
          libjetsonpower_handle_, "rail_get_crit_current");
      lib_rail_get_avg_power_ = (rail_get_avg_power_func)dlsym(
          libjetsonpower_handle_, "rail_get_avg_power");

      lib_fan_get_nums_ =
          (fan_get_nums_func)dlsym(libjetsonpower_handle_, "fan_get_nums");
      lib_fan_get_names_ =
          (fan_get_names_func)dlsym(libjetsonpower_handle_, "fan_get_names");
      lib_fan_get_pwm_ =
          (fan_get_pwm_func)dlsym(libjetsonpower_handle_, "fan_get_pwm");
      lib_fan_get_speed_ =
          (fan_get_speed_func)dlsym(libjetsonpower_handle_, "fan_get_speed");
      lib_fan_get_profile_ = (fan_get_profile_func)dlsym(libjetsonpower_handle_,
                                                         "fan_get_profile");

      lib_sensor_get_nums_ = (sensor_get_nums_func)dlsym(
          libjetsonpower_handle_, "thermal_get_sensor_nums");
      lib_sensor_get_names_ = (sensor_get_names_func)dlsym(
          libjetsonpower_handle_, "thermal_get_sensor_names");
      lib_sensor_get_temp_ = (sensor_get_temp_func)dlsym(
          libjetsonpower_handle_, "thermal_get_sensor_temp");
      lib_sensor_get_sw_throt_temp_ = (sensor_get_temp_func)dlsym(
          libjetsonpower_handle_, "thermal_get_sensor_sw_throttling_temp");
      lib_sensor_get_sw_shutdown_temp_ =
          (sensor_get_sw_shutdown_temp_func)dlsym(
              libjetsonpower_handle_, "thermal_get_sensor_sw_shutdown_temp");
    }
  }

  static std::shared_ptr<JetsonPowerLib> Instance() {
    pthread_once(&once, &JetsonPowerLib::init);
    return jetsonpower_lib;
  }

  int JetsonPower_sensor_get_nums() {
    if (lib_sensor_get_nums_) {
      return lib_sensor_get_nums_();
    }

    return -1;
  }

  char** JetsonPower_sensor_get_names() {
    if (lib_sensor_get_names_) {
      return lib_sensor_get_names_();
    }

    return nullptr;
  }

  int JetsonPower_sensor_get_temp(char* name) {
    if (lib_sensor_get_temp_) {
      return lib_sensor_get_temp_(name);
    }

    return -1;
  }

  int JetsonPower_sensor_get_sw_throt_temp(char* name) {
    if (lib_sensor_get_sw_throt_temp_) {
      return lib_sensor_get_sw_throt_temp_(name);
    }

    return -1;
  }

  int JetsonPower_sensor_get_sw_shutdown_temp(char* name) {
    if (lib_sensor_get_sw_shutdown_temp_) {
      return lib_sensor_get_sw_shutdown_temp_(name);
    }

    return -1;
  }

  int JetsonPower_fan_get_nums() {
    if (lib_fan_get_nums_) {
      return lib_fan_get_nums_();
    }

    return -1;
  }

  char** JetsonPower_fan_get_names() {
    if (lib_fan_get_names_) {
      return lib_fan_get_names_();
    }

    return nullptr;
  }

  int JetsonPower_fan_get_pwm(int id) {
    if (lib_fan_get_pwm_) {
      return lib_fan_get_pwm_(id);
    }

    return -1;
  }

  int JetsonPower_fan_get_speed(int id) {
    if (lib_fan_get_speed_) {
      return lib_fan_get_speed_(id);
    }

    return -1;
  }

  char* JetsonPower_fan_get_profile(int id) {
    if (lib_fan_get_profile_) {
      return lib_fan_get_profile_(id);
    }

    return nullptr;
  }

  int JetsonPower_rail_get_avg_power(char* name) {
    if (lib_rail_get_avg_power_) {
      return lib_rail_get_avg_power_(name);
    }

    return -1;
  }

  int JetsonPower_rail_get_crit_current(char* name) {
    if (lib_rail_get_crit_current_) {
      return lib_rail_get_crit_current_(name);
    }

    return -1;
  }

  int JetsonPower_rail_get_warn_current(char* name) {
    if (lib_rail_get_warn_current_) {
      return lib_rail_get_warn_current_(name);
    }

    return -1;
  }

  int JetsonPower_rail_get_voltage(char* name) {
    if (lib_rail_get_voltage_) {
      return lib_rail_get_voltage_(name);
    }

    return -1;
  }

  int JetsonPower_rail_get_power(char* name) {
    if (lib_rail_get_power_) {
      return lib_rail_get_power_(name);
    }

    return -1;
  }

  int JetsonPower_rail_get_current(char* cur) {
    if (lib_rail_get_current_) {
      return lib_rail_get_current_(cur);
    }

    return -1;
  }

  int JetsonPower_rail_get_nums() {
    if (lib_rail_get_nums_) {
      return lib_rail_get_nums_();
    }

    return -1;
  }

  char** JetsonPower_rail_get_names() {
    if (lib_rail_get_names_) {
      return lib_rail_get_names_();
    }

    return nullptr;
  }

  int JetsonPower_emc_get_swap_free_size() {
    if (lib_emc_get_swap_free_size_) {
      return lib_emc_get_swap_free_size_();
    }

    return -1;
  }

  int JetsonPower_emc_get_swap_size() {
    if (lib_emc_get_swap_size_) {
      return lib_emc_get_swap_size_();
    }

    return -1;
  }

  int JetsonPower_emc_get_cached_size() {
    if (lib_emc_get_cached_size_) {
      return lib_emc_get_cached_size_();
    }

    return -1;
  }

  int JetsonPower_emc_get_buffers_size() {
    if (lib_emc_get_buffers_size_) {
      return lib_emc_get_buffers_size_();
    }

    return -1;
  }

  int JetsonPower_emc_get_mem_size() {
    if (lib_emc_get_mem_size_) {
      return lib_emc_get_mem_size_();
    }

    return -1;
  }

  int JetsonPower_emc_get_mem_free_size() {
    if (lib_emc_get_mem_free_size_) {
      return lib_emc_get_mem_free_size_();
    }

    return -1;
  }

  int JetsonPower_emc_get_load() {
    if (lib_emc_get_load_) {
      return lib_emc_get_load_();
    }

    return -1;
  }

  long JetsonPower_emc_get_cur_freq() {
    if (lib_emc_get_cur_freq_) {
      return lib_emc_get_cur_freq_();
    }

    return -1;
  }

  long JetsonPower_igpu_get_cur_freq(int gpu) {
    if (lib_igpu_get_cur_freq_) {
      return lib_igpu_get_cur_freq_(gpu);
    }

    return -1;
  }

  int JetsonPower_igpu_get_load(int gpu) {
    if (lib_igpu_get_load_) {
      return lib_igpu_get_load_(gpu);
    }

    return -1;
  }

  bool is_loaded = false;

 private:
  using libjetson_init_func = int (*)(void);
  using igpu_get_cur_freq_func = long (*)(int);
  using igpu_get_load_func = int (*)(int);

  using emc_get_cur_freq_func = long (*)(void);
  using emc_get_load_func = int (*)(void);
  using emc_get_mem_size_func = int (*)(void);
  using emc_get_mem_free_size_func = int (*)(void);
  using emc_get_buffers_size_func = int (*)(void);
  using emc_get_cached_size_func = int (*)(void);
  using emc_get_swap_size_func = int (*)(void);
  using emc_get_swap_free_size_func = int (*)(void);

  using rail_get_nums_func = int (*)(void);
  using rail_get_names_func = char** (*)(void);
  using rail_get_current_func = int (*)(char*);
  using rail_get_power_func = int (*)(char*);
  using rail_get_voltage_func = int (*)(char*);
  using rail_get_warn_current_func = int (*)(char*);
  using rail_get_crit_current_func = int (*)(char*);
  using rail_get_avg_power_func = int (*)(char*);

  using fan_get_nums_func = int (*)(void);
  using fan_get_names_func = char** (*)(void);
  using fan_get_pwm_func = int (*)(int);
  using fan_get_speed_func = int (*)(int);
  using fan_get_profile_func = char* (*)(int);

  using sensor_get_nums_func = int (*)(void);
  using sensor_get_names_func = char** (*)(void);
  using sensor_get_temp_func = int (*)(char*);
  using sensor_get_sw_throt_temp_func = int (*)(char*);
  using sensor_get_sw_shutdown_temp_func = int (*)(char*);

  static void init() { jetsonpower_lib = std::make_shared<JetsonPowerLib>(); }
  static pthread_once_t once;
  static std::shared_ptr<JetsonPowerLib> jetsonpower_lib;
  void* libjetsonpower_handle_;

  igpu_get_cur_freq_func lib_igpu_get_cur_freq_;
  igpu_get_load_func lib_igpu_get_load_;

  emc_get_cur_freq_func lib_emc_get_cur_freq_;
  emc_get_load_func lib_emc_get_load_;

  emc_get_mem_size_func lib_emc_get_mem_size_;
  emc_get_mem_free_size_func lib_emc_get_mem_free_size_;
  emc_get_buffers_size_func lib_emc_get_buffers_size_;
  emc_get_cached_size_func lib_emc_get_cached_size_;
  emc_get_swap_size_func lib_emc_get_swap_size_;
  emc_get_swap_free_size_func lib_emc_get_swap_free_size_;

  rail_get_nums_func lib_rail_get_nums_;
  rail_get_names_func lib_rail_get_names_;
  rail_get_current_func lib_rail_get_current_;
  rail_get_power_func lib_rail_get_power_;
  rail_get_voltage_func lib_rail_get_voltage_;
  rail_get_warn_current_func lib_rail_get_warn_current_;
  rail_get_crit_current_func lib_rail_get_crit_current_;
  rail_get_avg_power_func lib_rail_get_avg_power_;

  fan_get_nums_func lib_fan_get_nums_;
  fan_get_names_func lib_fan_get_names_;
  fan_get_pwm_func lib_fan_get_pwm_;
  fan_get_speed_func lib_fan_get_speed_;
  fan_get_profile_func lib_fan_get_profile_;

  sensor_get_nums_func lib_sensor_get_nums_;
  sensor_get_names_func lib_sensor_get_names_;
  sensor_get_temp_func lib_sensor_get_temp_;
  sensor_get_sw_throt_temp_func lib_sensor_get_sw_throt_temp_;
  sensor_get_sw_shutdown_temp_func lib_sensor_get_sw_shutdown_temp_;
};

}  // namespace system_monitor
}  // namespace cargo
