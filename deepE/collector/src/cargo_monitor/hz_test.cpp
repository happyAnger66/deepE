#include "cargo_monitor/hz.h"
#include <glog/logging.h>
#include <iomanip>
#include <iostream>
#include <string>
#include "shm_stats/shm_hz.h"

#include "ros/ros.h"

int main(int argc, char *argv[]) {
  if (argc < 2) {
    std::cout << "usage: " << argv[0] << " <topic>" << std::endl;
    return -1;
  }

  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging("hz_test");
  google::InstallFailureSignalHandler();

  ros::init(argc, argv, "hz_test");
  ros::NodeHandle nh;
  std::string topic(argv[1]);

  ros::Rate rate_hz(1);
  shm_stats::TimeHz time_hz;
  while (ros::ok()) {
    rate_hz.sleep();
    if (shm_stats::HzStatus::NORMAL !=
        cargo::system_monitor::HzStats::Instance()->GetHz(topic, time_hz)) {
      std::cout << "no new messages" << std::endl;
      continue;
    }
    std::cout << "average rate: " << std::setprecision(3) << time_hz.rate
              << std::endl;
    std::cout << "\t" << std::setprecision(3) << "min: " << time_hz.min_delta
              << "s max: " << time_hz.max_delta
              << "s std_dev: " << std::setprecision(5) << time_hz.std_dev
              << "s window: " << time_hz.window << std::endl;
  }
}