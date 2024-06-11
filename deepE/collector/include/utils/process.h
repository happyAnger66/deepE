#pragma once
#include <sys/types.h>
#include <unistd.h>
#include <boost/chrono.hpp>
#include <string>

namespace cargo {
namespace system_monitor {
class Process {
 public:
  /* /proc/<pid>/statm
    #  ============================================================
    # | FIELD  | DESCRIPTION                         | AKA  | TOP  |
    #  ============================================================
    # | rss    | resident set size                   |      | RES  |
    # | vms    | total program size                  | size | VIRT |
    # | shared | shared pages (from shared mappings) |      | SHR  |
    # | text   | text ('code')                       | trs  | CODE |
    # | lib    | library (unused in Linux 2.6)       | lrs  |      |
    # | data   | data + stack                        | drs  | DATA |
    # | dirty  | dirty pages (unused in Linux 2.6)   | dt   |      |
    #  ============================================================
    */
  struct MemInfo {
    int64_t rss;
    int64_t vms;
    int64_t shared;
    int64_t text;
    int64_t lib;
    int64_t data;
    int64_t dirty;
  };

  /*
    rchar: read or pread bytes
    wchar: write or pwrite bytes
    syscr: io read system calls
    syscw: io write system calls
    read_bytes: read done with submit_io
    write_bytes: write done with submit_io
    cancelled_write_bytes: 0
    */

  struct IoCounter {
    int64_t r_char;
    int64_t w_char;
    int64_t syscall_r;
    int64_t syscall_w;
    int64_t read_bytes;
    int64_t write_bytes;
    int64_t cancel_write_bytes;
    boost::chrono::steady_clock::time_point time_stamp;
  };

  struct CpuTime {
    std::string name;
    std::string status;
    pid_t ppid;
    double utime;
    double stime;
    double children_utime;
    double children_stime;
    int64_t create_time;
    long policy;
    double io_delay;
    int64_t minflt;
    int64_t majflt;
    int processor;
    boost::chrono::steady_clock::time_point time_stamp;
  };

  struct ProcStatus {
    int threads;
    int rss_shmem;
    int vm_swap;
  };

  struct SchedInfo {
    int voluntary_ctx_switches;
    int nonvoluntary_ctx_switches;
    std::string policy;
    int prio;
  };

  struct SchedTime {
    int64_t run_times;
    int64_t wait_times;
    int64_t run_cnts;
    boost::chrono::steady_clock::time_point time_stamp;
  };

 public:
  explicit Process(pid_t pid) : pid_(pid) {}
  Process() {
    pid_ = getpid();
  }

  static long PageSize() {
    static long page_size = -1;
    if (page_size == -1) {
      page_size = sysconf(_SC_PAGESIZE);
    }

    return page_size;
  }

  static long TicksPerSecond() {
    static long ticks_per_second = -1;
    if (ticks_per_second == -1) {
      ticks_per_second = sysconf(_SC_CLK_TCK);
    }

    return ticks_per_second;
  }

  static std::string SchedName(int sched) {
    switch (sched) {
      case 0:
        return "NORMAL";
        break;
      case 1:
        return "FIFO";
        break;
      case 2:
        return "RR";
        break;
      case 3:
        return "BATCH";
        break;
      case 5:
        return "IDLE";
        break;
      case 6:
        return "DEADLINE";
        break;
      default:
        return "UNKOWN";
        break;
    }
  }

  bool CpuTimes(CpuTime &cm);
  bool SchedTimes(SchedTime &st);
  bool IoCounters(IoCounter &io);
  bool Status(ProcStatus &status);
  bool GetSchedInfo(SchedInfo &si);
  double CpuPercent();
  double MemPercent();

 private:
  int64_t mem_total();
  boost::chrono::steady_clock::time_point last_time_;
  CpuTime last_cpu_time_;
  pid_t pid_;
};
}  // namespace system_monitor
}  // namespace cargo
