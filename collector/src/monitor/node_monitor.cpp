#include "monitor/node_monitor.h"

#include <dirent.h>
#include <sys/types.h>

#include <string>

#include "utils/proc_stat.h"
#include "utils/process.h"
#include "utils/ros_utils.h"
#include "utils/utils.h"

namespace cargo {
namespace system_monitor {

constexpr static double kCpuUsedThreshold = 1.5;
constexpr static double kMemUsedThreshold = 0.2;
constexpr static double kIoDelayThreshold = 0;
constexpr static int32_t kMaxRecordProcessNum = 200;
constexpr static int32_t kMaxAllProcessNum = 600;
static constexpr int64_t kNsToMS = 1000 * 1000;

bool NodeMonitor::is_digits(const std::string &str) {
  return str.find_first_not_of("0123456789") == std::string::npos;
}

bool NodeMonitor::AddSchedTimes(const std::string &node_name, pid_t pid,
                                cargo::proto::NodeInfo *one_node_msg) {
  Process process(pid);
  Process::SchedTime sched_time;
  if (!process.SchedTimes(sched_time)) {
    return false;
  }

  auto old_iter = nodes_sched_time_.find(pid);
  if (old_iter != nodes_sched_time_.end()) {
    Process::SchedTime &sched_time_old = (*old_iter).second;
    double delta_time =
        Utils::SteadyTimeToS(sched_time.time_stamp, sched_time_old.time_stamp);

    double delta_run_times =
        (sched_time.run_times - sched_time_old.run_times) / kNsToMS;
    double delta_run_cnts = sched_time.run_cnts - sched_time_old.run_cnts;
    double delta_wait_times =
        (sched_time.wait_times - sched_time_old.wait_times) / kNsToMS;

    if (delta_time > 0) {
      float run_times_percent = delta_run_times / delta_time;
      float run_wait_times_percent = delta_wait_times / delta_time;
      float run_cnts_speed = delta_run_cnts / delta_time;

      float wait_times_percent = (delta_wait_times * 100) / (delta_time * 1000);
      one_node_msg->set_sched_run_time(run_times_percent);
      one_node_msg->set_sched_wait_time(run_wait_times_percent);
      one_node_msg->set_sched_run_cnts(run_cnts_speed);
      one_node_msg->set_cpu_wait_percent(wait_times_percent);
    }
  }

  nodes_sched_time_[pid] = sched_time;
  return true;
}

cargo::proto::NodeInfo *NodeMonitor::AddMemUsed(
    const std::string &node_name, pid_t pid, cargo::proto::SystemInfo *msg,
    cargo::proto::NodeInfo *one_node_msg) {
  Process process(pid);
  cargo::proto::NodeInfo *node_msg = one_node_msg;
  double mem_percent = process.MemPercent();
  if (mem_percent > kMemUsedThreshold) {
    if (!node_msg) {
      node_msg = msg->add_node_info();
    }
    node_msg->set_mem_used_percent(process.MemPercent());
    return node_msg;
  }

  return node_msg;
}

bool NodeMonitor::AddIoStat(const std::string &node_name, pid_t pid,
                            cargo::proto::NodeInfo *msg) {
  Process process(pid);
  Process::IoCounter io_counter;
  process.IoCounters(io_counter);

  if (!process.IoCounters(io_counter)) {
    return false;
  }

  bool ret = false;
  auto old_iter = nodes_io_.find(pid);
  if (old_iter != nodes_io_.end()) {
    Process::IoCounter &io_counter_old = (*old_iter).second;
    double delta_time =
        Utils::SteadyTimeToS(io_counter.time_stamp, io_counter_old.time_stamp);
    if (delta_time > 0) {
      double delta_write_kb =
          (io_counter.write_bytes - io_counter_old.write_bytes) / 1024;  // KB/s
      double delta_read_kb =
          (io_counter.read_bytes - io_counter_old.read_bytes) / 1024;  // KB/s
      double write_kb_speed = delta_write_kb / delta_time;
      double read_kb_speed = delta_read_kb / delta_time;
      msg->set_read_kb_speed(read_kb_speed);
      msg->set_write_kb_speed(write_kb_speed);
      msg->set_read_mbytes(static_cast<float>(io_counter.read_bytes) / 1024 /
                           1024);
      msg->set_write_mbytes(static_cast<float>(io_counter.write_bytes) / 1024 /
                            1024);
      ret = true;
    }
  }

  nodes_io_[pid] = io_counter;
  return ret;
}

cargo::proto::NodeInfo *NodeMonitor::AddCpuTime(const std::string &node_name,
                                                pid_t pid,
                                                cargo::proto::SystemInfo *msg) {
  Process process(pid);
  Process::CpuTime cpu_time_now{};
  if (!process.CpuTimes(cpu_time_now)) {
    return nullptr;
  }

  cargo::proto::NodeInfo *one_node_msg = nullptr;
  auto old_iter = nodes_cpu_.find(pid);
  if (old_iter != nodes_cpu_.end()) {
    Process::CpuTime &cpu_time_old = (*old_iter).second;
    double delta_time =
        Utils::SteadyTimeToS(cpu_time_now.time_stamp, cpu_time_old.time_stamp);
    if (delta_time > 0) {
      double delta_cpu_time = cpu_time_now.stime - cpu_time_old.stime +
                              cpu_time_now.utime - cpu_time_old.utime;
      double cpu_used_percent = delta_cpu_time / delta_time * 100.0;
      double delta_io_delay = cpu_time_now.io_delay - cpu_time_old.io_delay;
      float io_delay = delta_io_delay;  // ms

      double delta_sys_cpu = cpu_time_now.stime - cpu_time_old.stime;
      double delta_user_cpu = cpu_time_now.utime - cpu_time_old.utime;
      float cpu_user_percent = delta_user_cpu / delta_time * 100.0;
      float cpu_sys_percent = delta_sys_cpu / delta_time * 100.0;

      if (cpu_used_percent > kCpuUsedThreshold ||
          io_delay > kIoDelayThreshold) {
        one_node_msg = msg->add_node_info();
        one_node_msg->set_status(cpu_time_now.status);
        one_node_msg->set_cpu_used_percent(cpu_used_percent);
        one_node_msg->set_cpu_user_percent(cpu_user_percent);
        one_node_msg->set_cpu_sys_percent(cpu_sys_percent);
        one_node_msg->set_io_delay(io_delay);

        float minflt = (cpu_time_now.minflt - cpu_time_old.minflt) / delta_time;
        float majflt = (cpu_time_now.majflt - cpu_time_old.majflt) / delta_time;
        one_node_msg->set_minflt(minflt);
        one_node_msg->set_majflt(majflt);
        one_node_msg->set_processor(cpu_time_now.processor);
      }
    }
  }

  nodes_cpu_[pid] = cpu_time_now;
  return one_node_msg;
}

bool NodeMonitor::AddProcStatus(const std::string &node_name, pid_t pid,
                                cargo::proto::NodeInfo *msg) {
  bool ret = false;
  Process process(pid);
  Process::ProcStatus proc_status{};
  if (process.Status(proc_status)) {
    msg->set_threads(proc_status.threads);
    msg->set_rss_shmem(proc_status.rss_shmem / 1000.0);
    msg->set_vm_swap(proc_status.vm_swap / 1000.0);
  }

  Process::SchedInfo sched_info{};
  if (process.GetSchedInfo(sched_info)) {
    msg->set_sched_policy(sched_info.policy);
    msg->set_sched_prio(sched_info.prio);
    msg->set_voluntary_ctxt_switches(sched_info.voluntary_ctx_switches);
    msg->set_nonvoluntary_ctxt_switches(sched_info.nonvoluntary_ctx_switches);
  }

  return true;
}

int32_t NodeMonitor::RunOnce(cargo::proto::SystemInfo &msg) {
  TRACE_EVENT("app", "NodeMonitor::RunOnce");
  DIR *proc_dir = opendir("/proc");
  if (proc_dir == NULL) {
    return -1;
  }

  int32_t record_count = 0;
  int32_t all_count = 0;

  struct dirent *entry;
  while ((entry = readdir(proc_dir)) != NULL) {
    std::string pid_str(entry->d_name);
    if (!is_digits(pid_str)) {
      continue;
    }

    if (all_count++ > kMaxAllProcessNum) {
      break;
    }

    int pid = std::stoi(pid_str, nullptr);
    std::string proc_name;
    ProcStat::Comm(pid, proc_name);
    if (!ProcStat::Comm(pid, proc_name)) {
      continue;
    } else {
      auto one_node_msg = AddCpuTime(proc_name, pid, &msg);
      one_node_msg = AddMemUsed(proc_name, pid, &msg, one_node_msg);
      if (one_node_msg) {
        one_node_msg->set_name(proc_name);
        one_node_msg->set_pid(pid);
        AddIoStat(proc_name, pid, one_node_msg);
        AddProcStatus(proc_name, pid, one_node_msg);
        AddSchedTimes(proc_name, pid, one_node_msg);

        record_count += 1;
        if (record_count > kMaxRecordProcessNum) {
          break;
        }
      }
    }

    // auto one_node_msg = AddCpuTime(proc_name, pid, msg);
    // one_node_msg = AddMemUsed(proc_name, pid, msg, one_node_msg);
    // if (one_node_msg) {
    //   one_node_msg->set_name(proc_name);
    //   one_node_msg->set_pid(pid);
    //   AddIoStat(proc_name, pid, *one_node_msg);
    //   AddProcStatus(proc_name, pid, *one_node_msg);
    // }
  }

  closedir(proc_dir);
  return 0;
}

}  // namespace system_monitor
}  // namespace cargo
