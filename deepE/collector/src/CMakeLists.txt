# The light monitor entry.
set(CARGO_MONITOR_SRCS
  cargo_monitor.cpp
  system_monitor_node.cpp
  monitor/connection_monitor.cpp
  monitor/cpu_monitor.cpp
  monitor/cpu_temp_monitor.cpp
  monitor/msglat_monitor.cpp
  monitor/fs_monitor.cpp
  monitor/mem_monitor.cpp
  monitor/monitor_item.cpp
  monitor/msglat_monitor.cpp
  monitor/net_monitor.cpp
  monitor/node_monitor.cpp
  monitor/topic_monitor.cpp
  monitor/softirq_monitor.cpp
  monitor/status_monitor.cpp
  monitor/udp_monitor.cpp
  monitor/jetson_gpu_monitor.cpp
  monitor/gpu_monitor.cpp
  monitor/node_latency_monitor.cpp
  monitor/thread_monitor.cpp
  utils/netlink_helper.cpp
  utils/netlink_utils.cpp
  utils/proc_stat.cpp
  utils/process.cpp
  utils/thread.cpp
  utils/ros_utils.cpp
  utils/utils.cpp
  utils/netlink_utils.cpp
  utils/libsensors_chip.cpp
  utils/jetsonpower.cpp
)

add_executable(cargo_monitor_node
  ${CARGO_MONITOR_SRCS}
)

target_include_directories(cargo_monitor_node PUBLIC
  ${PROJECT_SOURCE_DIR}/..
)

find_package(CUDAToolkit REQUIRED)
if(BUILD_ASAN MATCHES ON)
  target_compile_options(cargo_monitor_node PUBLIC -fsanitize=address -O1 -g -fno-omit-frame-pointer)
  target_link_libraries(cargo_monitor_node PUBLIC -fsanitize=address -O1 -g -fno-omit-frame-pointer)
endif()

target_link_libraries(cargo_monitor_node PUBLIC
  cargo::common::proto
  cargo::module
  cargo::monitor
  cargo::shm
  voy::common::common
  cargo::system_monitor::proto
  cargo::system_monitor_config::proto
  cargo::log
  alpas::proto::bywire_protos
  voy::common::proto
  alpas::proto::platoon_protos
  service_manager::client
  cargo::tracing
  CUDA::nvml
)

add_library(cargo_monitor
  cargo_monitor/cargo_monitor.cpp
  cargo_monitor/cargo_monitor_cfg.cpp
  cargo_monitor/hz.cpp
  cargo_monitor/msg_lat_cfg.cpp
  cargo_monitor/topic_cfg.cpp
  cargo_monitor/topics_graph_cfg.cpp
  cargo_monitor/topics_tracer.cpp
)

target_include_directories(cargo_monitor PUBLIC
  ${PROJECT_SOURCE_DIR}/include) 

target_link_libraries(cargo_monitor PUBLIC
  cargo::system_monitor::proto
  cargo::system_monitor_config::proto
  cargo::task
  cargo::shm
  voy::common::common
  voy::base
  cargo::startup
)

add_library(cargo::monitor ALIAS cargo_monitor)

add_executable(hz_test
  cargo_monitor/hz_test.cpp)

target_include_directories(hz_test PUBLIC
  include
)
target_include_directories(hz_test SYSTEM PUBLIC
  ${catkin_INCLUDE_DIRS}
)

target_link_libraries(hz_test
  glog
  voy_ros_util
  cargo::common::proto
  cargo::shm
  cargo::module
)

add_subdirectory(test)

#------------- Install -------------#
install(TARGETS cargo_monitor_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS cargo_monitor
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
