#pragma once

#include <algorithm>
#include <flatbuffers/flatbuffers.h>
#include <std_msgs/String.h>
#include <std_msgs_string_generated.h>

#include <vector>

static std::vector<uint8_t> encode(const std_msgs::String& msg) {
  StdMsgsStringT obj;
  obj.data = msg.data;
  flatbuffers::FlatBufferBuilder fbb;
  fbb.Finish(StdMsgsString::Pack(fbb, &obj));

  std::vector<uint8_t> data(fbb.GetSize());
  std::copy(fbb.GetBufferPointer(), fbb.GetBufferPointer() + fbb.GetSize(), data.begin());
  return data;
}

static std_msgs::String decode_string(const uint8_t* const data,
                                      const size_t& size) {
  const StdMsgsString* str = GetStdMsgsString(data);
  std_msgs::String msg;
  msg.data = str->data()->str();
  return msg;
}
