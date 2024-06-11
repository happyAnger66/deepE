#pragma once

#include <memory>
#include <string>

#include "common_protos/input_info.pb.h"

namespace cargo {
namespace system_monitor {

template <class...>
using void_t = void;

template <class, class = void>
struct has_input_infos : std::false_type {};

template <class ProtoType>
struct has_input_infos<
    ProtoType, void_t<decltype(std::declval<ProtoType>().input_infos())>>
    : std::true_type {};

template <class, class = void>
struct has_header : std::false_type {};

template <class ProtoType>
struct has_header<ProtoType,
                  void_t<decltype(std::declval<ProtoType>().header())>>
    : std::true_type {};

template <class MsgType,
          std::enable_if_t<has_input_infos<MsgType>::value, bool> = true>
int32_t TraceInputMsg(MsgType *msg, const cargo::common::InputInfos &inputs) {
  *(msg->mutable_input_infos()) = inputs;
  return 0;
}

}  // namespace system_monitor
}  // namespace cargo
