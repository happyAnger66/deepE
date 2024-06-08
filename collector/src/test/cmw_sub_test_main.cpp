#include "cmw_sub_test_node.h"

#include <base/base_dir.h>
#include <base/now.h>
#include <glog/logging.h>

#include "log/async_logsink_entry.h"

int main(int argc, char **argv) {
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  cargo::common::logsink::InitAsyncLogSink(argv[0]);

  std::string cfg_file = base::GetBaseDirPath(base::BaseDir::kConfigDir) +
                         "/config/node/cmw_sub_test/sub_test.pb.conf";

  cargo::system_monitor::CmwSubNode node(cfg_file);
  node.Init();
  node.Start();

  //  ros::spin();
  cargo::WaitForShutdown();

  return 0;
}
