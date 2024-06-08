#include "cargo_monitor/cargo_monitor.h"

int main() {
  cargo::system_monitor::TopicHzInfo info;
  if (!SystemMonitorGetAllTopicsHz(info)) {
    printf("Get topics hz failed\r\n");
    return -1;
  }

  for (auto it = info.cbegin(); it != info.cend(); it++) {
    printf("topic: %s\n", it->first.c_str());

    auto one_info = it->second;
    printf("name: %s\n", one_info.name.c_str());
    printf("normal_hz: %f\n", one_info.hz_normal);
    printf("cur_hz: %f\n", one_info.hz_cur);
    printf("\n\n");
  }

  return 0;
}