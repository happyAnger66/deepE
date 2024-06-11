#include <dirent.h>
#include <glog/logging.h>
#include <sys/types.h>

#include "cargo_monitor/cargo_monitor.h"
#include "cargo_monitor/hz.h"
#include "cargo_monitor/topic_cfg.h"

namespace cargo {
namespace system_monitor {

bool endsWith(const std::string &str, const std::string &suffix) {
  if (str.size() < suffix.size()) {
    return false;
  }

  std::string estr = str.substr(str.size() - suffix.size());
  return estr == suffix;
}

bool get_all_shm_topics(std::vector<std::string> &topics) {
  DIR *shm_dir = opendir("/dev/shm");
  if (shm_dir == NULL) {
    return false;
  }

  struct dirent *entry;
  std::string suffix("_hz_stats");
  while ((entry = readdir(shm_dir)) != NULL) {
    std::string shm_name(entry->d_name);

    if (shm_name == "." || shm_name == "..") {
      continue;
    }

    if (!endsWith(shm_name, suffix)) {
      continue;
    }

    replace(shm_name.begin(), shm_name.end(), ':', '/');
    std::string topic = shm_name.substr(0, shm_name.size() - suffix.size());
    topics.push_back(topic);
  }

  closedir(shm_dir);
  return true;
}

bool SystemMonitorGetAllTopicsHz(TopicHzInfo &topic_hz_info) {
  auto topic_cfg =
      cargo::system_monitor::TopicCfg::Instance()->GetAllTopicCfgs();

  std::vector<std::string> cur_topics;
  if (!get_all_shm_topics(cur_topics)) {
    LOG(INFO) << "get_all_shm_topics failed !";
    return false;
  }

  for (auto &topic : cur_topics) {
    OneTopicHz one_hz;
    one_hz.name = topic;
    if (topic_cfg.find(topic) != topic_cfg.end()) {
      auto cfg = topic_cfg[topic];
      one_hz.hz_normal = cfg.hz();
    }

    shm_stats::TimeHz hz;
    auto hz_status =
        cargo::system_monitor::HzStats::Instance()->GetHz(one_hz.name, hz);
    if (hz_status == shm_stats::HzStatus::NORMAL) {
      one_hz.hz_cur = hz.rate;
    } else {
      one_hz.hz_cur = 0;
    }

    topic_hz_info[one_hz.name] = one_hz;
  }

  return true;
}

}  // namespace system_monitor
}  // namespace cargo
