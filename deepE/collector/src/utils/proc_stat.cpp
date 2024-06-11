#include <cstdlib>
#include <strstream>

#include <fstream>
#include "base/now.h"
#include "utils/proc_file.h"
#include "utils/proc_stat.h"

namespace cargo {
namespace system_monitor {

static constexpr char kProcStatFile[] = "/proc/stat";

int ProcStat::AllCpuStat(std::vector<CpuStat> &all_cpu_stat) {
  ProcFile proc_stat_file(kProcStatFile);

  while (true) {
    CpuStat stat{};
    bool ret = proc_stat_file.ReadOneLine(stat.cpu_info, stat.user, stat.nice,
                                          stat.system, stat.idle, stat.io_wait,
                                          stat.irq, stat.soft_irq, stat.steal,
                                          stat.guest, stat.guest_nice);
    if (!ret || stat.cpu_info.find("cpu") != 0) {
      break;
    }
    all_cpu_stat.push_back(stat);
  }

  return 0;
}

int ProcStat::GetSysInfo(SysInfo &sys_info) {
  ProcFile proc_stat_file(kProcStatFile);
  std::vector<std::string> oneLine;
  while (true) {
    bool ret = proc_stat_file.ReadOneLine(oneLine);
    if (!ret) {
      return -1;
    }

    if (oneLine[0] == "ctxt") {
      sys_info.cs = std::stoll(oneLine[1]);
    }

    if (oneLine[0] == "processes") {
      sys_info.procs = std::stoll(oneLine[1]);
    }

    if (oneLine[0] == "procs_running") {
      sys_info.procs_running = std::stoll(oneLine[1]);
    }

    if (oneLine[0] == "procs_blocked") {
      sys_info.procs_blocked = std::stoll(oneLine[1]);
      break;
    }

    oneLine.clear();
  }

  sys_info.ts = base::NowNs();
  return 0;
}

bool ProcStat::Stat(CpuStat &stat) {
  ProcFile proc_stat_file(kProcStatFile);

  return proc_stat_file.ReadOneLine(
      stat.cpu_info, stat.user, stat.nice, stat.system, stat.idle, stat.io_wait,
      stat.irq, stat.soft_irq, stat.steal, stat.guest, stat.guest_nice);
}

bool ProcStat::Comm(int32_t pid, std::string &comm) {
  std::stringstream ss;
  ss << pid;
  std::string comm_file =
      std::string("/proc/") + ss.str() + std::string("/comm");

  ProcFile proc_comm_file(comm_file);
  return proc_comm_file.ReadOneLine(comm);
}

}  // namespace system_monitor
}  // namespace cargo
