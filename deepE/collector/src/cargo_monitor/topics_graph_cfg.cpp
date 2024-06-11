#include "cargo_monitor/topics_graph_cfg.h"

template <>
std::once_flag
    av_comm::Singleton<cargo::system_monitor::TopicsGraphCfg>::flag{};

template <>
std::unique_ptr<cargo::system_monitor::TopicsGraphCfg>
    av_comm::Singleton<cargo::system_monitor::TopicsGraphCfg>::instance_ =
        nullptr;
