#pragma once

#include <math.h>
#include <pthread.h>
#include <mutex>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include "shm_stats/shm_hz.h"
#include "topic_cfg.h"

namespace cargo {
namespace system_monitor {

class HzStats {
 public:
  HzStats() = default;
  static std::shared_ptr<HzStats> Instance() {
    pthread_once(&once, &HzStats::init);
    return hz_stats;
  }

  int32_t AddSample(const std::string &topic, double timestamp);
  shm_stats::HzStatus GetHz(const std::string &topic,
                            shm_stats::TimeHz &time_hz);
  inline bool IsHzEq(double d1, double d2, double precision) {
    return fabs(d1 - d2) < (d2 * (precision / 100.0));
  }
  shm_stats::HzStatus GetStatus(const std::string &topic);

 private:
  static void init() { hz_stats = std::make_shared<HzStats>(); }
  int32_t calc_window_by_normal(int32_t normal_hz);
  static pthread_once_t once;
  static std::shared_ptr<HzStats> hz_stats;
  std::mutex mutex_;
  std::map<std::string, std::shared_ptr<shm_stats::ShmHz>> pub_hz_;
  std::map<std::string, std::shared_ptr<shm_stats::ShmHz>> sub_hz_;
  std::map<std::string, std::pair<double, double>> hz_cfg_;
};

}  // namespace system_monitor
}  // namespace cargo
