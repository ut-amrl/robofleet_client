#include <gtest/gtest.h>

#include <std_msgs/Header.h>
#include "schema_generated.h"
#include "../src/decode.hpp"

static void encode_std_header(flatbuffers::FlatBufferBuilder& fbb) {
  const fb::RosTime ros_time(200, 300);
  auto fb_header = fb::std_msgs::CreateHeaderDirect(
    fbb,
    0,
    100,
    &ros_time,
    "frame"
  );
  fbb.Finish(fb_header);
}

TEST(DecodeObjectTest, HandlesStdHeader) {
  flatbuffers::FlatBufferBuilder fbb;
  encode_std_header(fbb);

  const fb::std_msgs::Header* encoded = flatbuffers::GetRoot<fb::std_msgs::Header>(fbb.GetBufferPointer());
  const std_msgs::Header decoded = decode_obj<std_msgs::Header>(encoded);

  EXPECT_EQ(decoded.frame_id, "frame");
  EXPECT_EQ(decoded.seq, 100);
  EXPECT_EQ(decoded.stamp.sec, 200);
  EXPECT_EQ(decoded.stamp.nsec, 300);
}
