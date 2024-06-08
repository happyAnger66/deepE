#include "utils/jetsonpower.h"

namespace cargo {
namespace system_monitor {

pthread_once_t JetsonPowerLib::once = PTHREAD_ONCE_INIT;
std::shared_ptr<JetsonPowerLib> JetsonPowerLib::jetsonpower_lib = nullptr;

}  // namespace system_monitor
}  // namespace cargo
