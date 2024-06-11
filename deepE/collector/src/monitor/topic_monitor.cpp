#include "monitor/topic_monitor.h"
#include <iomanip>
#include "shm_stats/shm_hz.h"

namespace cargo {
namespace system_monitor {

int32_t TopicMonitor::Start(uint32_t interval_ms) {
  for (const auto &one_topic : cfg_.topic()) {
    all_monitor_topics_[one_topic.name()] = one_topic;
  }

  hz_stats_ = cargo::system_monitor::HzStats::Instance();

  (void)interval_ms;
  return 0;
}

int32_t TopicMonitor::RunOnce(cargo::proto::SystemInfo &msg) {
  TRACE_EVENT("app", "TopicMonitor::RunOnce");
  for (auto iter = all_monitor_topics_.begin();
       iter != all_monitor_topics_.end(); iter++) {
    TopicInfo info{};
    auto topic = iter->second;
    info.name = topic.name();
    info.cfg_hz = topic.hz();
    info.error = topic.error();

    shm_stats::TimeHz hz;
    shm_stats::HzStatus status = hz_stats_->GetHz(topic.name(), hz);
    if (status == shm_stats::HzStatus::NORMAL) {
      info.cur_hz = hz.rate;
      info.min_delta = hz.min_delta;
      info.max_delta = hz.max_delta;
      info.std_dev = hz.std_dev;
      info.window = hz.window;

      if (!hz_stats_->IsHzEq(info.cur_hz, info.cfg_hz, info.error)) {
        std::stringstream ss;
        auto one_status = msg.add_status_list();

        ss << "topic: " << info.name << " current hz: [ "
           << std::setprecision(3) << info.cur_hz
           << " ] is different with config hz: [ " << info.cfg_hz
           << " ] greater than an allowed mean error: [ " << info.error
           << "% ]";

        one_status->set_msg(ss.str());
        one_status->set_error_code(cargo::common::ErrorCode::TOPIC_HZ_ERROR);
      }
    } else if (status == shm_stats::HzStatus::NO_NEW) {
      std::stringstream ss;
      auto one_status = msg.add_status_list();

      ss << "topic: " << info.name << " have no new message !!!";
      one_status->set_msg(ss.str());
      one_status->set_error_code(cargo::common::ErrorCode::TOPIC_HZ_ERROR);
    } else if (status == shm_stats::HzStatus::TIMEOUT) {
      std::stringstream ss;
      auto one_status = msg.add_status_list();

      ss << "topic: " << info.name
         << " have no new message timeout !!! publisher may be exited !!!";
      one_status->set_msg(ss.str());
      one_status->set_error_code(cargo::common::ErrorCode::TOPIC_HZ_ERROR);
    } else if (status == shm_stats::HzStatus::EMPTY) {
      std::stringstream ss;
      auto one_status = msg.add_status_list();

      ss << "topic: " << info.name
         << " have no message !!! publisher may not published yet !!!";
      one_status->set_msg(ss.str());
      one_status->set_error_code(cargo::common::ErrorCode::TOPIC_HZ_ERROR);
    }
  }

  return 0;
}

}  // namespace system_monitor
}  // namespace cargo
