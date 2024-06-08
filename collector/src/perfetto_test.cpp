#include <chrono>
#include <thread>
#include "tracing/tracer.h"

using namespace std::chrono_literals;

PERFETTO_DEFINE_CATEGORIES(
    perfetto::Category("rendering")
        .SetDescription("Events from the graphics subsystem"),
    perfetto::Category("network").SetDescription(
        "Network upload and download statistics"));

PERFETTO_TRACK_EVENT_STATIC_STORAGE();

int main(int argc, char** argv) {
  perfetto::TracingInitArgs args;

  // The backends determine where trace events are recorded. You may select one
  // or more of:

  // 1) The in-process backend only records within the app itself.
  args.backends |= perfetto::kInProcessBackend;

  // 2) The system backend writes events into a system Perfetto daemon,
  //    allowing merging app and system events (e.g., ftrace) on the same
  //    timeline. Requires the Perfetto `traced` daemon to be running (e.g.,
  //    on Android Pie and newer).
  args.backends |= perfetto::kSystemBackend;

  perfetto::Tracing::Initialize(args);
  perfetto::TrackEvent::Register();

  {
    TRACE_EVENT("rendering", "LayerTreeHost::DoUpdateLayers");
    std::this_thread::sleep_for(10000ms);
  }
  return 0;
}
