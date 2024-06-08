#pragma once
#include <sys/types.h>
#include <unistd.h>
#include <boost/chrono.hpp>
#include <string>

namespace cargo {
namespace system_monitor {
class Thread {
 public:
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
    int64_t time_stamp;
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
    int processor;
    long policy;
    double io_delay;
    int64_t minflt;
    int64_t majflt;
    int64_t time_stamp;
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
    int64_t time_stamp;
  };

 public:
  explicit Thread(pid_t pid, pid_t tid) : pid_(pid), tid_(tid) {}

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
  bool GetSchedInfo(SchedInfo &si);
  double CpuPercent();

 private:
  pid_t pid_;
  pid_t tid_;
};

}  // namespace system_monitor
}  // namespace cargo
