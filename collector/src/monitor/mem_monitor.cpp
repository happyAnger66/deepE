#include <sys/sysinfo.h>

#include "monitor/mem_monitor.h"
#include "utils/proc_file.h"

namespace cargo {
namespace system_monitor {

static constexpr int KKBToGB = 1000 * 1000;
static constexpr int KKBToMB = 1000;
int32_t MemMonitor::RunOnce(cargo::proto::SystemInfo &system_info_msg) {
  TRACE_EVENT("app", "MemMonitor::RunOnce");
  ProcFile mem_proc_file("/proc/meminfo");

  std::string item, mem_size;
  long long total, free, avail, buffers, cached;
  long long swap_cached = 0, active = 0, in_active = 0, active_anon = 0,
            inactive_anon = 0;
  long long active_file = 0, inactive_file = 0, dirty = 0, writeback = 0,
            anon_pages = 0, mapped = 0;
  long long kReclaimable = 0, sReclaimable = 0, sUnreclaim = 0;

  while (mem_proc_file.ReadOneLine(item, mem_size)) {
    if (item == "MemTotal:") {
      total = std::stoll(mem_size);
    } else if (item == "MemFree:") {
      free = std::stoll(mem_size);
    } else if (item == "MemAvailable:") {
      avail = std::stoll(mem_size);
    } else if (item == "Buffers:") {
      buffers = std::stoll(mem_size);
    } else if (item == "Cached:") {
      cached = std::stoll(mem_size);
    } else if (item == "SwapCached:") {
      swap_cached = std::stoll(mem_size);
    } else if (item == "Active:") {
      active = std::stoll(mem_size);
    } else if (item == "Inactive:") {
      in_active = std::stoll(mem_size);
    } else if (item == "Active(anon):") {
      active_anon = std::stoll(mem_size);
    } else if (item == "Inactive(anon):") {
      inactive_anon = std::stoll(mem_size);
    } else if (item == "Active(file):") {
      active_file = std::stoll(mem_size);
    } else if (item == "Inactive(file):") {
      inactive_file = std::stoll(mem_size);
    } else if (item == "Dirty:") {
      dirty = std::stoll(mem_size);
    } else if (item == "Writeback:") {
      writeback = std::stoll(mem_size);
    } else if (item == "AnonPages:") {
      anon_pages = std::stoll(mem_size);
    } else if (item == "Mapped:") {
      mapped = std::stoll(mem_size);
    } else if (item == "KReclaimable:") {
      kReclaimable = std::stoll(mem_size);
    } else if (item == "SReclaimable:") {
      sReclaimable = std::stoll(mem_size);
    } else if (item == "SUnreclaim:") {
      sUnreclaim = std::stoll(mem_size);
    }
  }

  double usage_percent =
      static_cast<double>(total - avail) / static_cast<double>(total) * 100.0;

  system_info_msg.set_mem_free_size(static_cast<float>(avail) / KKBToGB);
  system_info_msg.set_mem_used_percent(usage_percent);
  system_info_msg.set_mem_avail_size(
      static_cast<float>(free) /
      KKBToGB);  // For compatible, we set avail to free. cviz and monitor treat
                 // avail as free.
  system_info_msg.set_mem_buffers_size(static_cast<float>(buffers) / KKBToGB);
  system_info_msg.set_mem_cached_size(static_cast<float>(cached) / KKBToGB);

  auto mem_detail = system_info_msg.mutable_mem_detail();
  mem_detail->set_swap_cached(static_cast<float>(swap_cached) / KKBToMB);
  mem_detail->set_active(static_cast<float>(active) / KKBToMB);
  mem_detail->set_inactive(static_cast<float>(in_active) / KKBToMB);
  mem_detail->set_active_anon(static_cast<float>(active_anon) / KKBToMB);
  mem_detail->set_inactive_anon(static_cast<float>(inactive_anon) / KKBToMB);
  mem_detail->set_active_file(static_cast<float>(active_file) / KKBToMB);
  mem_detail->set_inactive_file(static_cast<float>(inactive_file) / KKBToMB);
  mem_detail->set_dirty(static_cast<float>(dirty) / KKBToMB);
  mem_detail->set_writeback(static_cast<float>(writeback) / KKBToMB);
  mem_detail->set_anon_pages(static_cast<float>(anon_pages) / KKBToMB);
  mem_detail->set_mapped(static_cast<float>(mapped) / KKBToMB);
  mem_detail->set_kreclaimable(static_cast<float>(kReclaimable) / KKBToMB);
  mem_detail->set_sreclaimable(static_cast<float>(sReclaimable) / KKBToMB);
  mem_detail->set_sunreclaim(static_cast<float>(sUnreclaim) / KKBToMB);

  return 0;
}

}  // namespace system_monitor
}  // namespace cargo