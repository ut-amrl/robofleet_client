#include <gtest/gtest.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>

#include "../../robofleet_client_lib/include/schema_generated.h"
#include "../../src/encode_ros.hpp"
#include "../../src/decode_ros.hpp"

TEST(EncodeDecode, PointCloud2) {
  sensor_msgs::PointCloud2 src;
  src.height = 10;
  src.is_bigendian = true;
  src.is_dense = true;
  src.point_step = 20;
  src.row_step = 30;
  src.width = 40;

  sensor_msgs::PointField src_field1;
  src_field1.count = 50;
  src_field1.datatype = sensor_msgs::PointField::UINT32;
  src_field1.name = "test";
  src_field1.offset = 1;
  src.fields.push_back(src_field1);

  sensor_msgs::PointField src_field2;
  src_field1.count = 60;
  src_field1.datatype = sensor_msgs::PointField::UINT32;
  src_field1.name = "test2";
  src_field1.offset = 2;
  src.fields.push_back(src_field2);

  src.data.insert(src.data.end(), {1u, 2u, 4u, 8u, 16u});

  // encode
  flatbuffers::FlatBufferBuilder fbb;
  auto encoded_offset = encode(fbb, src, 0);
  fbb.Finish(flatbuffers::Offset<void>(encoded_offset));

  // decode
  const fb::sensor_msgs::PointCloud2* root = flatbuffers::GetRoot<typename flatbuffers_type_for<sensor_msgs::PointCloud2>::type>(fbb.GetBufferPointer());
  const sensor_msgs::PointCloud2 decoded = decode<sensor_msgs::PointCloud2>(root);

  EXPECT_EQ(src, decoded);
}
