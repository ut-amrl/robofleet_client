#pragma once

#include <flatbuffers/flexbuffers.h>
#include <std_msgs/String.h>

#include <vector>

static std::vector<uint8_t> encode(const std_msgs::String& msg) {
  flexbuffers::Builder fbb;
  fbb.Map([&]() {
    fbb.String("data", msg.data);
  });
  fbb.Finish();
  return fbb.GetBuffer();
}

static std_msgs::String decode_string(const uint8_t* const data,
                                      const size_t& size) {
  flexbuffers::Map m = flexbuffers::GetRoot(data, size).AsMap();
  std_msgs::String msg;
  msg.data = m["data"].AsString().str();
  return msg;
}
