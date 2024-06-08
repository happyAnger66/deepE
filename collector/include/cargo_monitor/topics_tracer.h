#pragma once

#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <utility>

#include <glog/logging.h>

#include "common_protos/input_info.pb.h"

#include "base/macros.h"
#include "base/now.h"

#include "cargo_monitor/msg_trace.h"

namespace cargo {
namespace system_monitor {

constexpr int MAX_TOPIC_TRACER_SIZE = 100;

struct TimePoints {
  int64_t recv_time;
  int64_t callback_time;
  int64_t callback_end_time;
};

class TopicTracer {
 public:
  template <
      class MsgType,
      std::enable_if_t<!cargo::system_monitor::has_input_infos<MsgType>::value,
                       bool> = true>
  int32_t AppendInputTopicInfos(const std::string & /*topic*/,
                                MsgType * /*msg*/) {
    return 0;
  }

  template <
      class MsgType,
      std::enable_if_t<cargo::system_monitor::has_input_infos<MsgType>::value,
                       bool> = true>
  int32_t AppendInputTopicInfos(const std::string & /*topic*/, MsgType *msg) {
    if (!msg->has_header()) {
      auto add_header = msg->mutable_header();
      add_header->set_timestamp_ns(base::NowNs());
    }

    if (!msg->has_input_infos()) {
      return 0;
    }

    auto input_infos = msg->mutable_input_infos();
    int info_size = input_infos->input_info_size();
    for (int i = 0; i < info_size; i++) {
      auto input_info = input_infos->mutable_input_info(i);
      add_input_recv_timestamp(input_info);
    }

    return 0;
  }

  template <class MsgType,
            std::enable_if_t<!cargo::system_monitor::has_header<MsgType>::value,
                             bool> = true>
  int32_t ReceiveInputTopics(const std::string & /*topic*/, MsgType * /*msg*/,
                             TimePoints /*time_points*/) {
    return 0;
  }

  template <class MsgType,
            std::enable_if_t<cargo::system_monitor::has_header<MsgType>::value,
                             bool> = true>
  int32_t ReceiveInputTopics(const std::string &topic, MsgType *msg,
                             TimePoints time_points) {
    {
      std::lock_guard<std::mutex> lock_guard(lock);
      sub_msgs_[topic].push_back({msg->header().timestamp_ns(), time_points});

      auto &sub_msgs_queue = sub_msgs_[topic];
      if (sub_msgs_queue.size() > MAX_TOPIC_TRACER_SIZE) {
        sub_msgs_queue.pop_front();
      }
    }
    return 0;
  }

 private:
  std::mutex lock;
  std::map<std::string, std::deque<std::pair<int64_t, TimePoints>>> sub_msgs_;

  int add_input_recv_timestamp(cargo::common::InputInfo *msg) {
    const std::string topic_name = msg->topic_name();
    int64_t timestamp_ns = msg->header().timestamp_ns();

    // LOG(INFO) << "add recv" << topic_name;
    std::lock_guard<std::mutex> lock_guard(lock);
    auto &sub_msgs_queue = sub_msgs_[topic_name];
    for (auto &sub_msg : sub_msgs_queue) {
      // LOG(INFO) << "first " << sub_msg.first << "second " << sub_msg.second
      //           << " time: " << timestamp_ns;
      if (sub_msg.first == timestamp_ns) {
        msg->set_recv_timestamp_ns(sub_msg.second.recv_time);
        msg->set_callback_timestamp_ns(sub_msg.second.callback_time);
        msg->set_callback_end_timestamp_ns(sub_msg.second.callback_end_time);
        break;
      }
    }

    return 0;
  }
  DECLARE_SINGLETON(TopicTracer)
};

}  // namespace system_monitor
}  // namespace cargo
