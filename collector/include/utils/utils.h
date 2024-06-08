#pragma once

#include <string>

#include <boost/chrono.hpp>

namespace cargo {
namespace system_monitor {
class Utils {
 public:
  static double SteadyTimeToS(
      const boost::chrono::steady_clock::time_point &t1,
      const boost::chrono::steady_clock::time_point &t2) {
    boost::chrono::duration<double> sec = t1 - t2;
    return sec.count();
  }
};

namespace utils {
std::string GetFileInTripPath(const std::string &module_name);
}
}  // namespace system_monitor
}  // namespace cargo
