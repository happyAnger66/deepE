#include <cstdlib>
#include <iostream>

#include "absl/strings/str_cat.h"

#include "base/now.h"
#include "utils/proc_file.h"
#include "utils/thread.h"

namespace cargo {
namespace system_monitor {

bool Thread::SchedTimes(SchedTime &st) {
  std::string stat_file =
      absl::StrCat("/proc/", pid_, "/task/", tid_, "/schedstat");
  ProcFile stat_proc_file(stat_file);

  std::vector<std::string> datas;
  if (!stat_proc_file.ReadOneLine(datas)) {
    return false;
  }

  st.time_stamp = base::NowNs();
  st.run_times = std::stoll(datas[0]);
  st.wait_times = std::stoll(datas[1]);
  st.run_cnts = std::stoll(datas[2]);
  return true;
}

bool Thread::CpuTimes(CpuTime &cm) {
  std::string stat_file = absl::StrCat("/proc/", pid_, "/task/", tid_, "/stat");
  ProcFile stat_proc_file(stat_file);

  std::vector<std::string> datas;
  if (!stat_proc_file.ReadOneLine(datas)) {
    return false;
  }

  cm.time_stamp = base::NowNs();
  cm.name = datas[1];
  cm.status = datas[2];
  cm.ppid = std::stoll(datas[3]);
  cm.minflt = std::stol(datas[9]);
  cm.majflt = std::stol(datas[11]);
  cm.utime = static_cast<double>(std::stoll(datas[13])) / TicksPerSecond();
  cm.stime = static_cast<double>(std::stoll(datas[14])) / TicksPerSecond();
  cm.children_utime =
      static_cast<double>(std::stoll(datas[15])) / TicksPerSecond();
  cm.children_stime =
      static_cast<double>(std::stoll(datas[16])) / TicksPerSecond();
  cm.create_time = std::stoll(datas[21]);
  cm.processor = std::stoi(datas[38]);
  cm.policy = std::stol(datas[40]);
  cm.io_delay = static_cast<double>(std::stol(datas[41]));
  return true;
}

bool Thread::IoCounters(IoCounter &io_counter) {
  std::string io_file = absl::StrCat("/proc/", pid_, "/task/", tid_, "/io");
  ProcFile io_proc_file(io_file);

  std::vector<std::string> all_lines;
  while (io_proc_file.ReadOneLine(all_lines)) {
    if (all_lines[0] == "rchar:") {
      io_counter.r_char = std::stoll(all_lines[1]);
    } else if (all_lines[0] == "wchar:") {
      io_counter.w_char = std::stoll(all_lines[1]);
    } else if (all_lines[0] == "syscr:") {
      io_counter.syscall_r = std::stoll(all_lines[1]);
    } else if (all_lines[0] == "syscw:") {
      io_counter.syscall_w = std::stoll(all_lines[1]);
    } else if (all_lines[0] == "read_bytes:") {
      io_counter.read_bytes = std::stoll(all_lines[1]);
    } else if (all_lines[0] == "write_bytes:") {
      io_counter.write_bytes = std::stoll(all_lines[1]);
    } else if (all_lines[0] == "cancelled_write_bytes:") {
      io_counter.cancel_write_bytes = std::stoll(all_lines[1]);
    }
    all_lines.clear();
  }

  io_counter.time_stamp = base::NowNs();
  return true;
}

bool Thread::GetSchedInfo(SchedInfo &si) {
  std::string sched_file =
      absl::StrCat("/proc/", pid_, "/task/", tid_, "/sched");
  ProcFile sched_proc_file(sched_file);

  std::vector<std::string> all_lines;
  int match_num = 0;
  while (sched_proc_file.ReadOneLine(all_lines) && match_num < 4) {
    if (all_lines[0] == "nr_voluntary_switches") {
      si.voluntary_ctx_switches = std::stoi(all_lines[2]);
    } else if (all_lines[0] == "nr_involuntary_switches") {
      si.nonvoluntary_ctx_switches = std::stoi(all_lines[2]);
    } else if (all_lines[0] == "policy") {
      si.policy = SchedName(std::stoi(all_lines[2]));
    } else if (all_lines[0] == "prio") {
      si.prio = std::stoll(all_lines[2]);
    } else {
      all_lines.clear();
      continue;
    }

    match_num++;
    all_lines.clear();
  }

  return true;
}

}  // namespace system_monitor
}  // namespace cargo
