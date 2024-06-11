#include <sstream>

#include "common_protos/status.pb.h"
#include "monitor/status_monitor.h"

namespace cargo {
namespace system_monitor {

int32_t StatusMonitor::Start(uint32_t interval_ms) {
  system_cpu_cfg_.high = cfg_.system_cpu().high_percent();
  system_cpu_cfg_.critical = cfg_.system_cpu().critical_percent();

  system_mem_cfg_.high = cfg_.system_mem().high_percent();
  system_mem_cfg_.critical = cfg_.system_mem().critical_percent();

  default_cpu_cfg_.high = cfg_.default_cpu().high_percent();
  default_cpu_cfg_.critical = cfg_.default_cpu().critical_percent();

  default_mem_cfg_.high = cfg_.default_mem().high_percent();
  default_mem_cfg_.critical = cfg_.default_mem().critical_percent();

  for (auto &node_cfg : cfg_.module()) {
    threshold_value cpu{};
    cpu.high = node_cfg.cpu().high_percent();
    cpu.critical = node_cfg.cpu().critical_percent();
    nodes_cpu_cfg_[node_cfg.name()] = cpu;

    threshold_value mem{};
    mem.high = node_cfg.mem().high_percent();
    mem.critical = node_cfg.mem().critical_percent();
    nodes_mem_cfg_[node_cfg.name()] = mem;
  }

  cpu_temp_.high = cfg_.cpu_temp().high();
  cpu_temp_.critical = cfg_.cpu_temp().critical();
  disk_usage_.high = cfg_.fs_cfg().high();
  disk_usage_.critical = cfg_.fs_cfg().critical();

  gpu_temp_.high = cfg_.gpu_config().temp_high();
  gpu_temp_.critical = cfg_.gpu_config().temp_critical();
  gpu_pwr_.high = cfg_.gpu_config().pwr_high();
  gpu_pwr_.critical = cfg_.gpu_config().pwr_critical();
  gpu_mem_.high = cfg_.gpu_config().mem_high();
  gpu_mem_.critical = cfg_.gpu_config().mem_critical();

  (void)interval_ms;
  return 0;
}

bool StatusMonitor::system_check(cargo::proto::SystemInfo &msg) {
  float cpu_percent = msg.cpu_used_percent();
  float mem_percent = msg.mem_used_percent();

  if (cpu_percent > system_cpu_cfg_.critical) {
    std::stringstream ss;
    ss << "System cpu [CRITICAL] warning: " << cpu_percent
       << "% is higher than critical cfg: " << system_cpu_cfg_.critical << "% "
       << std::endl;
    auto status_msg = msg.add_status_list();
    status_msg->set_msg(ss.str());
    status_msg->set_error_code(cargo::common::SYSTEM_CPU_CRITICAL);
  } else if (cpu_percent > system_cpu_cfg_.high) {
    std::stringstream ss;
    ss << "System cpu [HIGH] warning: " << cpu_percent
       << "% is higher than high cfg: " << system_cpu_cfg_.high << "% "
       << std::endl;
    auto status_msg = msg.add_status_list();
    status_msg->set_msg(ss.str());
    status_msg->set_error_code(cargo::common::SYSTEM_CPU_HIGH);
  }

  if (mem_percent > system_mem_cfg_.critical) {
    std::stringstream ss;
    ss << "System memory [CRITICAL] warning: " << mem_percent
       << "% is higher than critical cfg: " << system_mem_cfg_.critical << "% "
       << std::endl;
    auto status_msg = msg.add_status_list();
    status_msg->set_msg(ss.str());
    status_msg->set_error_code(cargo::common::SYSTEM_MEM_CRITICAL);
  } else if (mem_percent > system_mem_cfg_.high) {
    std::stringstream ss;
    ss << "System memory [HIGH] warning: " << mem_percent
       << "% is higher than high cfg: " << system_mem_cfg_.high << "% "
       << std::endl;
    auto status_msg = msg.add_status_list();
    status_msg->set_msg(ss.str());
    status_msg->set_error_code(cargo::common::SYSTEM_MEM_HIGH);
  }

  return true;
}

bool StatusMonitor::node_default_cpu_check(const cargo::proto::NodeInfo &node,
                                           cargo::proto::SystemInfo &msg) {
  float node_cpu = node.cpu_used_percent();

  if (node_cpu > default_cpu_cfg_.critical) {
    std::stringstream ss;
    ss << "node: " << node.name()
       << " cpu [CRITICAL] used percent: " << node_cpu
       << "% is higher than default critical cpu: " << default_cpu_cfg_.critical
       << "% " << std::endl;

    auto status_msg = msg.add_status_list();
    status_msg->set_msg(ss.str());
    status_msg->set_error_code(cargo::common::NODE_CPU_CRITICAL);
  } else if (node_cpu > default_cpu_cfg_.high) {
    std::stringstream ss;
    ss << "node: " << node.name() << " cpu [HIGH] used percent: " << node_cpu
       << "% is higher than default high cpu: " << default_cpu_cfg_.high << "% "
       << std::endl;

    auto status_msg = msg.add_status_list();
    status_msg->set_msg(ss.str());
    status_msg->set_error_code(cargo::common::NODE_CPU_HIGH);
  }

  return true;
}

bool StatusMonitor::node_default_mem_check(const cargo::proto::NodeInfo &node,
                                           cargo::proto::SystemInfo &msg) {
  float node_mem = node.mem_used_percent();

  if (node_mem > default_mem_cfg_.critical) {
    std::stringstream ss;
    ss << "node: " << node.name()
       << " mem [CRITICAL] used percent: " << node_mem
       << "% is higher than default mem: " << default_mem_cfg_.critical << "% "
       << std::endl;

    auto status_msg = msg.add_status_list();
    status_msg->set_msg(ss.str());
    status_msg->set_error_code(cargo::common::NODE_MEM_CRITICAL);
  } else if (node_mem > default_mem_cfg_.high) {
    std::stringstream ss;
    ss << "node: " << node.name() << " mem [HIGH] used percent: " << node_mem
       << "% is higher than default mem: " << default_mem_cfg_.high << "% "
       << std::endl;

    auto status_msg = msg.add_status_list();
    status_msg->set_msg(ss.str());
    status_msg->set_error_code(cargo::common::NODE_MEM_HIGH);
  }

  return true;
}

bool StatusMonitor::node_cpu_check(const cargo::proto::NodeInfo &node,
                                   cargo::proto::SystemInfo &msg,
                                   threshold_value &threshold) {
  float node_cpu = node.cpu_used_percent();

  if (node_cpu > threshold.critical) {
    std::stringstream ss;
    ss << "node: " << node.name()
       << " cpu [CRITICAL] used percent: " << node_cpu
       << "% is higher than node critical cpu: " << threshold.critical << "% "
       << std::endl;

    auto status_msg = msg.add_status_list();
    status_msg->set_msg(ss.str());
    status_msg->set_error_code(cargo::common::NODE_CPU_CRITICAL);
  } else if (node_cpu > threshold.high) {
    std::stringstream ss;
    ss << "node: " << node.name() << " cpu [HIGH] used percent: " << node_cpu
       << "% is higher than node high cpu: " << threshold.high << "% "
       << std::endl;

    auto status_msg = msg.add_status_list();
    status_msg->set_msg(ss.str());
    status_msg->set_error_code(cargo::common::NODE_CPU_HIGH);
  }

  return true;
}

bool StatusMonitor::node_mem_check(const cargo::proto::NodeInfo &node,
                                   cargo::proto::SystemInfo &msg,
                                   threshold_value &threshold) {
  float node_mem = node.mem_used_percent();

  if (node_mem > threshold.critical) {
    std::stringstream ss;
    ss << "node: " << node.name()
       << " mem [CRITICAL] used percent: " << node_mem
       << "% is higher than node mem critical: " << threshold.critical << "% "
       << std::endl;

    auto status_msg = msg.add_status_list();
    status_msg->set_msg(ss.str());
    status_msg->set_error_code(cargo::common::NODE_MEM_CRITICAL);
  } else if (node_mem > threshold.high) {
    std::stringstream ss;
    ss << "node: " << node.name() << " mem [HIGH] used percent: " << node_mem
       << "% is higher than node mem high: " << threshold.high << "% "
       << std::endl;

    auto status_msg = msg.add_status_list();
    status_msg->set_msg(ss.str());
    status_msg->set_error_code(cargo::common::NODE_MEM_HIGH);
  }

  return true;
}

bool StatusMonitor::find_cfg_by_name(
    const std::unordered_map<std::string, threshold_value> &cfg,
    const std::string &name, threshold_value &val) {
  for (auto iter = cfg.begin(); iter != cfg.end(); iter++) {
    std::string cfg_name = iter->first;
    if (name.find(cfg_name) != std::string::npos) {
      val = iter->second;
      return true;
    }
  }

  return false;
}

bool StatusMonitor::node_check(cargo::proto::SystemInfo &msg) {
  for (auto &node_msg : msg.node_info()) {
    std::string name = node_msg.name();
    threshold_value cpu_cfg;
    if (!find_cfg_by_name(nodes_cpu_cfg_, name, cpu_cfg)) {
      node_default_cpu_check(node_msg, msg);
    } else {
      node_cpu_check(node_msg, msg, cpu_cfg);
    }

    threshold_value mem_cfg;
    if (!find_cfg_by_name(nodes_mem_cfg_, name, mem_cfg)) {
      node_default_mem_check(node_msg, msg);
    } else {
      node_mem_check(node_msg, msg, mem_cfg);
    }
  }

  return true;
}

bool StatusMonitor::cpu_temp_check(cargo::proto::SystemInfo &msg) {
  for (const auto &cpu_temp_msg : msg.cpu_temp()) {
    if (cpu_temp_msg.temperature() > cpu_temp_.critical) {
      std::stringstream ss;
      ss << "cpu: " << cpu_temp_msg.description() << " temperature [CRITICAL] "
         << cpu_temp_msg.temperature()
         << " is higher than : " << cpu_temp_.critical << std::endl;

      auto status_msg = msg.add_status_list();
      status_msg->set_msg(ss.str());
      status_msg->set_error_code(cargo::common::SYSTEM_CPU_TEMP_CRITICAL);
    } else if (cpu_temp_msg.temperature() > cpu_temp_.high) {
      std::stringstream ss;
      ss << "cpu: " << cpu_temp_msg.description() << " temperature [HIGH] "
         << cpu_temp_msg.temperature() << " is higher than : " << cpu_temp_.high
         << std::endl;

      auto status_msg = msg.add_status_list();
      status_msg->set_msg(ss.str());
      status_msg->set_error_code(cargo::common::SYSTEM_CPU_TEMP_HIGH);
    }
  }

  return true;
}

bool StatusMonitor::disk_usage_check(cargo::proto::SystemInfo &msg) {
  for (const auto &fs_info : msg.filesystem_info()) {
    if (fs_info.used_percent() > disk_usage_.critical) {
      std::stringstream ss;
      ss << "disk: " << fs_info.mount_point() << " usage [CRITICAL] "
         << fs_info.used_percent()
         << " is higher than : " << disk_usage_.critical << std::endl;

      auto status_msg = msg.add_status_list();
      status_msg->set_msg(ss.str());
      status_msg->set_error_code(cargo::common::DISK_USAGE_CRITICAL);
    } else if (fs_info.used_percent() > disk_usage_.high) {
      std::stringstream ss;
      ss << "disk: " << fs_info.mount_point() << " usage [HIGH] "
         << fs_info.used_percent() << " is higher than : " << disk_usage_.high
         << std::endl;

      auto status_msg = msg.add_status_list();
      status_msg->set_msg(ss.str());
      status_msg->set_error_code(cargo::common::DISK_USAGE_HIGH);
    }
  }

  return true;
}

bool StatusMonitor::gpu_temp_check(const cargo::proto::GpuInfo &one_gpu,
                                   cargo::proto::SystemInfo &msg) {
  if (one_gpu.temperature() > gpu_temp_.critical) {
    std::stringstream ss;
    ss << "gpu: " << one_gpu.name() << " temperature [CRITICAL] "
       << one_gpu.temperature() << " is higher than : " << gpu_temp_.critical
       << std::endl;

    auto status_msg = msg.add_status_list();
    status_msg->set_msg(ss.str());
    status_msg->set_error_code(cargo::common::GPU_TEMP_CRITICAL);
  } else if (one_gpu.temperature() > gpu_temp_.high) {
    std::stringstream ss;
    ss << "gpu: " << one_gpu.name() << " temperature [HIGH] "
       << one_gpu.temperature() << " is higher than : " << gpu_temp_.high
       << std::endl;

    auto status_msg = msg.add_status_list();
    status_msg->set_msg(ss.str());
    status_msg->set_error_code(cargo::common::GPU_TEMP_HIGH);
  }

  return true;
}

bool StatusMonitor::gpu_pwr_check(const cargo::proto::GpuInfo &one_gpu,
                                  cargo::proto::SystemInfo &msg) {
  if (one_gpu.pwr_usage() > gpu_pwr_.critical) {
    std::stringstream ss;
    ss << "gpu: " << one_gpu.name() << " pwr [CRITICAL] " << one_gpu.pwr_usage()
       << " is higher than : " << gpu_pwr_.critical << std::endl;

    auto status_msg = msg.add_status_list();
    status_msg->set_msg(ss.str());
    status_msg->set_error_code(cargo::common::GPU_PWR_CRITICAL);
  } else if (one_gpu.pwr_usage() > gpu_pwr_.high) {
    std::stringstream ss;
    ss << "gpu: " << one_gpu.name() << " pwr [HIGH] " << one_gpu.pwr_usage()
       << " is higher than : " << gpu_pwr_.high << std::endl;

    auto status_msg = msg.add_status_list();
    status_msg->set_msg(ss.str());
    status_msg->set_error_code(cargo::common::GPU_PWR_HIGH);
  }

  return true;
}

bool StatusMonitor::gpu_mem_check(const cargo::proto::GpuInfo &one_gpu,
                                  cargo::proto::SystemInfo &msg) {
  if (one_gpu.mem_used_percent() > gpu_mem_.critical) {
    std::stringstream ss;
    ss << "gpu: " << one_gpu.name() << " memory [CRITICAL] "
       << one_gpu.mem_used_percent()
       << " is higher than : " << gpu_mem_.critical << std::endl;

    auto status_msg = msg.add_status_list();
    status_msg->set_msg(ss.str());
    status_msg->set_error_code(cargo::common::GPU_MEM_CRITICAL);
  } else if (one_gpu.mem_used_percent() > gpu_mem_.high) {
    std::stringstream ss;
    ss << "gpu: " << one_gpu.name() << " memory [HIGH] "
       << one_gpu.mem_used_percent() << " is higher than : " << gpu_mem_.high
       << std::endl;

    auto status_msg = msg.add_status_list();
    status_msg->set_msg(ss.str());
    status_msg->set_error_code(cargo::common::GPU_MEM_HIGH);
  }

  return true;
}

bool StatusMonitor::gpu_check(cargo::proto::SystemInfo &msg) {
  for (const auto &one_gpu : msg.gpu_info()) {
    gpu_temp_check(one_gpu, msg);
    gpu_pwr_check(one_gpu, msg);
    gpu_mem_check(one_gpu, msg);
  }

  return true;
}

int32_t StatusMonitor::RunOnce(cargo::proto::SystemInfo &msg) {
  TRACE_EVENT("app", "StatusMonitor::RunOnce");
  system_check(msg);
  node_check(msg);
  cpu_temp_check(msg);
  gpu_check(msg);
  disk_usage_check(msg);

  return 0;
}

}  // namespace system_monitor
}  // namespace cargo
