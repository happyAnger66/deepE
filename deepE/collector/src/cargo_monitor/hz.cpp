#include <glog/logging.h>

#include "cargo_monitor_protos/system_monitor_config.pb.h"

#include "cargo_monitor/hz.h"

namespace cargo {
namespace system_monitor {

pthread_once_t HzStats::once = PTHREAD_ONCE_INIT;
std::shared_ptr<HzStats> HzStats::hz_stats = nullptr;

static constexpr int32_t kWindowFactor = 5;
static constexpr int32_t kDefaultWindow = 100;

int32_t HzStats::calc_window_by_normal(int32_t normal_hz) {
  if (normal_hz == -1) {
    return kDefaultWindow;
  }
  return normal_hz * kWindowFactor;
}

int32_t HzStats::AddSample(const std::string &topic, double timestamp) {
  int32_t hz = -1;
  cargo::proto::TopicConfig cfg;
  if (TopicCfg::Instance()->GetTopicCfg(topic, cfg) == 0) {
    hz = static_cast<int32_t>(cfg.hz());
  }

  std::string hz_stats_name = topic + "_hz_stats";
  std::shared_ptr<shm_stats::ShmHz> hz_stat = nullptr;
  {
    std::lock_guard<std::mutex> lk(mutex_);
    auto hz_iter = pub_hz_.find(hz_stats_name);
    if (hz_iter == pub_hz_.end()) {
      hz_stat = std::make_shared<shm_stats::ShmHz>(
          hz_stats_name, shm_stats::ShmFlag::WRITE, calc_window_by_normal(hz));
      if (hz_stat == nullptr || 0 != hz_stat->Open()) {
        LOG(WARNING) << "failed to create hz stats " << topic.c_str();
        return -1;
      }

      pub_hz_[hz_stats_name] = hz_stat;
    }
    hz_stat = pub_hz_[hz_stats_name];
  }

  hz_stat->AddSample(timestamp);
  return 0;
}

shm_stats::HzStatus HzStats::GetStatus(const std::string &topic) {
  cargo::proto::TopicConfig cfg;
  if (TopicCfg::Instance()->GetTopicCfg(topic, cfg) != 0) {
    return shm_stats::HzStatus::NO_CFG;
  }

  shm_stats::TimeHz time_hz;
  shm_stats::HzStatus status;
  status = GetHz(topic, time_hz);
  if (status != shm_stats::HzStatus::NORMAL) {
    return status;
  }

  if (!IsHzEq(time_hz.rate, cfg.hz(), cfg.error())) {
    return shm_stats::HzStatus::ERROR;
  }

  return shm_stats::HzStatus::NORMAL;
}

shm_stats::HzStatus HzStats::GetHz(const std::string &topic,
                                   shm_stats::TimeHz &time_hz) {
  std::string hz_stats_name = topic + "_hz_stats";
  auto hz_iter = sub_hz_.find(hz_stats_name);
  std::shared_ptr<shm_stats::ShmHz> hz_stat = nullptr;
  if (hz_iter == sub_hz_.end()) {
    hz_stat = std::make_shared<shm_stats::ShmHz>(hz_stats_name,
                                                 shm_stats::ShmFlag::READ);
    if (0 != hz_stat->Open()) {
      LOG(WARNING) << "failed to read hz stats " << topic.c_str();
      return shm_stats::HzStatus::NOT_EXIST;
    }

    sub_hz_[hz_stats_name] = hz_stat;
  }

  shm_stats::HzStatus status = sub_hz_[hz_stats_name]->GetHz(time_hz);
  if (status == shm_stats::HzStatus::TIMEOUT) {
    sub_hz_.erase(hz_stats_name);
  }

  if (status != shm_stats::HzStatus::NORMAL) {
    /*LOG(WARNING) << "read hz stats not normal" << hz_stats_name
                 << " status:" << static_cast<int>(status);*/
  }

  return status;
}

}  // namespace system_monitor
}  // namespace cargo
