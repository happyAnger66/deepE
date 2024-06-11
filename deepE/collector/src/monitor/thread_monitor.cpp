#include <iomanip>
#include <vector>

#include <absl/strings/str_split.h>
#include <glog/logging.h>

#include "common_protos/status.pb.h"
#include "monitor/thread_monitor.h"
#include "utils/proc_stat.h"
#include "utils/ros_utils.h"

namespace cargo {
namespace system_monitor {

static constexpr int64_t KTimeOutInS = 10;
static constexpr int64_t KUsToMs = 1000;
static constexpr int64_t KUsToS = 1000 * 1000;
static constexpr int64_t kNsToS = 1000 * 1000 * 1000;
static constexpr int64_t kNsToMS = 1000 * 1000;
static constexpr int64_t kNsToUS = 1000;

using namespace shm_stats;

int32_t ThreadMonitor::Start(uint32_t interval_ms) {
  shm_thread_mgr_ = shm_stats::ShmThreadMgr::Instance();
  return 0;

  (void)interval_ms;
}

int32_t ThreadMonitor::Stop() {
  shm_thread_mgr_.reset();
  return 0;
}

cargo::proto::NodeThreadInfo *ThreadMonitor::AddCpuTime(
    pid_t pid, pid_t tid, int type, cargo::proto::SystemInfo &msg) {
  Thread thread(pid, tid);
  Thread::CpuTime cpu_time_now;
  if (!thread.CpuTimes(cpu_time_now)) {
    return nullptr;
  }

  cargo::proto::NodeThreadInfo *one_thread_msg = nullptr;
  auto old_iter = threads_cpu_.find(tid);
  if (old_iter != threads_cpu_.end()) {
    Thread::CpuTime &cpu_time_old = (*old_iter).second;
    double delta_time =
        (cpu_time_now.time_stamp - cpu_time_old.time_stamp) / kNsToS;
    if (delta_time > 0) {
      double delta_cpu_time = cpu_time_now.stime - cpu_time_old.stime +
                              cpu_time_now.utime - cpu_time_old.utime;
      double cpu_used_percent = delta_cpu_time / delta_time * 100.0;
      double delta_io_delay = cpu_time_now.io_delay - cpu_time_old.io_delay;
      float io_delay = delta_io_delay;  // ms

      double delta_sys_time = cpu_time_now.stime - cpu_time_old.stime;
      double delta_user_time = cpu_time_now.utime - cpu_time_old.utime;
      float cpu_user_percent = delta_user_time / delta_time * 100.0;
      float cpu_sys_percent = delta_sys_time / delta_time * 100.0;

      one_thread_msg = msg.add_node_thread_info();
      std::string comm{};
      if (pid != tid) {
        if (!ProcStat::Comm(pid, comm)) {
          comm = "";
        }
      }
      std::string thread_name = comm + cpu_time_now.name;
      one_thread_msg->set_name(thread_name);
      one_thread_msg->set_pid(pid);
      one_thread_msg->set_tid(tid);
      one_thread_msg->set_status(cpu_time_now.status);
      one_thread_msg->set_cpu_used_percent(cpu_used_percent);
      one_thread_msg->set_io_delay(io_delay);
      one_thread_msg->set_type(type);
      one_thread_msg->set_processor(cpu_time_now.processor);
      one_thread_msg->set_cpu_user_percent(cpu_user_percent);
      one_thread_msg->set_cpu_sys_percent(cpu_sys_percent);
    }
  }

  threads_cpu_[tid] = cpu_time_now;
  return one_thread_msg;
}

bool ThreadMonitor::AddIoStat(pid_t pid, pid_t tid,
                              cargo::proto::NodeThreadInfo &msg) {
  Thread thread(pid, tid);
  Thread::IoCounter io_counter;

  if (!thread.IoCounters(io_counter)) {
    return false;
  }

  bool ret = false;
  auto old_iter = threads_io_.find(tid);
  if (old_iter != threads_io_.end()) {
    Thread::IoCounter &io_counter_old = (*old_iter).second;
    double delta_time =
        (io_counter.time_stamp - io_counter_old.time_stamp) / kNsToS;
    if (delta_time > 0) {
      double delta_write_kb =
          (io_counter.write_bytes - io_counter_old.write_bytes) / 1024;  // KB/s
      double delta_read_kb =
          (io_counter.read_bytes - io_counter_old.read_bytes) / 1024;  // KB/s
      double write_kb_speed = delta_write_kb / delta_time;
      double read_kb_speed = delta_read_kb / delta_time;
      msg.set_read_kb_speed(read_kb_speed);
      msg.set_write_kb_speed(write_kb_speed);
      msg.set_read_mbytes(static_cast<float>(io_counter.read_bytes) / 1024 /
                          1024);
      msg.set_write_mbytes(static_cast<float>(io_counter.write_bytes) / 1024 /
                           1024);
      ret = true;
    }
  }

  threads_io_[tid] = io_counter;
  return ret;
}

bool ThreadMonitor::AddSchedTimes(pid_t pid, pid_t tid,
                                  cargo::proto::NodeThreadInfo &msg) {
  Thread thread(pid, tid);
  Thread::SchedTime sched_time;
  if (!thread.SchedTimes(sched_time)) {
    return false;
  }

  auto old_iter = threads_sched_time_.find(tid);
  if (old_iter != threads_sched_time_.end()) {
    Thread::SchedTime &sched_time_old = (*old_iter).second;
    double delta_time =
        (sched_time.time_stamp - sched_time_old.time_stamp) / kNsToS;
    double delta_run_times =
        (sched_time.run_times - sched_time_old.run_times) / kNsToMS;
    double delta_run_cnts = sched_time.run_cnts - sched_time_old.run_cnts;
    double delta_wait_times =
        (sched_time.wait_times - sched_time_old.wait_times) / kNsToMS;

    if (delta_time > 0) {
      float run_times_percent = delta_run_times / delta_time;
      float run_wait_times_percent = delta_wait_times / delta_time;
      float run_cnts_speed = delta_run_cnts / delta_time;

      msg.set_sched_run_time(run_times_percent);
      msg.set_sched_wait_time(run_wait_times_percent);
      msg.set_sched_run_cnts(run_cnts_speed);

      float wait_percent = (sched_time.wait_times - sched_time_old.wait_times) /
                           kNsToS / delta_time * 100.0;
      msg.set_cpu_wait_percent(wait_percent);
    }
  }

  threads_sched_time_[tid] = sched_time;
  return true;
}

bool ThreadMonitor::AddStatus(pid_t pid, pid_t tid,
                              cargo::proto::NodeThreadInfo &msg) {
  Thread thread(pid, tid);

  Thread::SchedInfo sched_info{};
  if (thread.GetSchedInfo(sched_info)) {
    msg.set_sched_policy(sched_info.policy);
    msg.set_sched_prio(sched_info.prio);
    msg.set_voluntary_ctxt_switches(sched_info.voluntary_ctx_switches);
    msg.set_nonvoluntary_ctxt_switches(sched_info.nonvoluntary_ctx_switches);
  }

  return true;
}

int32_t ThreadMonitor::RunOnce(cargo::proto::SystemInfo &msg) {
  TRACE_EVENT("app", "ThreadMonitor::RunOnce");
  ShmThreadMgr::ThreadShmVec vec;
  if (ShmThreadMgr::GetAllThreadShm(vec) != 0) {
    LOG(INFO) << "No Shm Records found";
    return -1;
  }

  std::vector<int> pids;
  std::vector<std::vector<std::pair<int, int>>> all_threads;

  for (auto &shm_name : vec) {
    int pid;
    std::vector<std::pair<int, int>> threads;
    if (shm_thread_mgr_->GetThreads(shm_name, threads, pid) != 0) {
      LOG(INFO) << "Get shm threads faile" << shm_name;
      continue;
    }

    pids.push_back(pid);
    all_threads.push_back(threads);
  }

  return AddAllThreads(pids, all_threads, msg);
}

int32_t ThreadMonitor::AddOneThread(int pid, int tid, int type,
                                    cargo::proto::SystemInfo &msg) {
  auto one_msg = AddCpuTime(pid, tid, type, msg);
  if (one_msg) {
    AddIoStat(pid, tid, *one_msg);
    AddSchedTimes(pid, tid, *one_msg);
    AddStatus(pid, tid, *one_msg);
  }

  return 0;
}

int32_t ThreadMonitor::AddOneProcessThreads(
    int pid, const std::vector<std::pair<int, int>> &threads,
    cargo::proto::SystemInfo &msg) {
  for (const auto &iter : threads) {
    AddOneThread(pid, iter.first, iter.second, msg);
  }

  return 0;
}

int32_t ThreadMonitor::AddAllThreads(
    const std::vector<int> &pids,
    const std::vector<std::vector<std::pair<int, int>>> &all_threads,
    cargo::proto::SystemInfo &msg) {
  int pids_len = pids.size();
  for (int i = 0; i < pids_len; i++) {
    AddOneProcessThreads(pids[i], all_threads[i], msg);
  }

  return 0;
}

}  // namespace system_monitor
}  // namespace cargo
