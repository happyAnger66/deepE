#pragma once
#include <stdint.h>

#include <string>
#include <vector>

namespace cargo {
namespace system_monitor {

class ProcStat {
 public:
  enum stat_item {
    CPU_NAME = 0,
    USER,
    SYSTEM,
    IDLE,
    NICE,
    IO_WAIT,
    IRQ,
    SOFT_IRQ,
    STEAL,
    GUEST,
    GUEST_NICE,
    ITEM_MAX
  };

  struct CpuStat {
    std::string cpu_info;
    int64_t user;
    int64_t system;
    int64_t idle;
    int64_t nice;
    int64_t io_wait;
    int64_t irq;
    int64_t soft_irq;
    int64_t steal;
    int64_t guest;
    int64_t guest_nice;
  };

  struct SysInfo {
    int64_t cs;
    int64_t procs;
    int64_t procs_running;
    int64_t procs_blocked;
    int64_t ts;
  };

  static bool Stat(CpuStat &cpu_total);
  static bool Comm(int32_t pid, std::string &comm);
  static int AllCpuStat(std::vector<CpuStat> &all_cpu_stat);
  static int GetSysInfo(SysInfo &sys_info);
};

}  // namespace system_monitor
}  // namespace cargo