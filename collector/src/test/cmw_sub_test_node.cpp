#include <fcntl.h>
#include <string>
#include <unordered_map>

#include <base/base_dir.h>
#include <base/now.h>
#include <common/status.h>
#include <glog/logging.h>
#include <module_protos/module_config.pb.h>

#include "cargo_monitor_protos/system_info.pb.h"

#include "cmw_sub_test_node.h"

namespace cargo {

namespace system_monitor {

bool CmwSubNode::Init() {
  std::string topic = "/system_info_orin_0";
  RegisterSub<cargo::proto::SystemInfo>(
      topic, [this](const cargo::proto::SystemInfo &msg) {
        std::cout << "msg " << msg.DebugString() << std::endl;
      });

  return true;
}

CmwSubNode::~CmwSubNode() {}

}  // namespace system_monitor
}  // namespace cargo
