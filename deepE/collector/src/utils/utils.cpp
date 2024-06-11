#include "service_manager_client/rpc_record_handle.h"

namespace cargo {
namespace system_monitor {
namespace utils {

std::string GetFileInTripPath(const std::string &module_name) {
  std::stringstream trace_path;
  auto t = std::time(nullptr);
  auto tm = *std::localtime(&t);

  service_manager::RpcRecordHandle recorder;
  service_manager::TripInfo trip_info = recorder.GetTripInfo();
  std::string trip_id = trip_info.id();

  if (trip_id.empty()) {
    trace_path << std::getenv("CARGO_DIR") << "/data/perf/" << module_name
               << "_protobuf_" << std::put_time(&tm, "%Y-%m-%d-%H-%M-%S")
               << ".log";
  } else {
    trace_path << "/bags/BAG_" << trip_id << "/logs/" << module_name
               << "_protobuf_" << std::put_time(&tm, "%Y-%m-%d-%H-%M-%S")
               << ".log";
  }

  return trace_path.str();
}

}  // namespace utils
}  // namespace system_monitor
}  // namespace cargo
