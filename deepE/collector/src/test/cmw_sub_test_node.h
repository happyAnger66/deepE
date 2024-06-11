#pragma once

#include <memory>
#include <vector>

#include "av_comm/topics.h"
#include "module/module.h"

#include "cmw_clcpp/cmw_clcpp.hpp"

namespace cargo {
namespace system_monitor {

class CmwSubNode : public Module {
 public:
  explicit CmwSubNode(const std::string &config_file) : Module(config_file){};
  ~CmwSubNode();

  virtual bool Init() override;

 private:
  DISALLOW_COPY_AND_ASSIGN(CmwSubNode);
};

}  // namespace system_monitor
}  // namespace cargo
