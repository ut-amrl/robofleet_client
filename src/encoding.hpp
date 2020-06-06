#pragma once

#include <algorithm>
#include <flatbuffers/flatbuffers.h>
#include <sensor_msgs/NavSatFix.h>
#include <schema_generated.h>

#include <vector>


static std::vector<uint8_t> encode(const sensor_msgs::NavSatFix& msg) {
  flatbuffers::FlatBufferBuilder fbb;

  auto header_stamp = fb::RosTime(msg.header.stamp.toSec(), msg.header.stamp.toNSec());
  auto header = fb::std_msgs::CreateHeaderDirect(
    fbb,
    0,
    msg.header.seq,
    &header_stamp,
    msg.header.frame_id.c_str()
  );
  
  auto status = fb::sensor_msgs::CreateNavSatStatus(
    fbb,
    0,
    msg.status.service,
    msg.status.status
  );

  auto metadata = fb::CreateMsgMetadataDirect(fbb, "/navsat/fix");
  auto covariance = fbb.CreateVector(msg.position_covariance.data(), msg.position_covariance.size());
  auto nav_sat_fix = fb::sensor_msgs::CreateNavSatFix(
    fbb,
    metadata,
    header,
    status,
    msg.latitude,
    msg.longitude,
    msg.altitude,
    covariance,
    msg.position_covariance_type
  );

  fbb.Finish(nav_sat_fix);

  std::vector<uint8_t> data(fbb.GetSize());
  std::copy(fbb.GetBufferPointer(), fbb.GetBufferPointer() + fbb.GetSize(), data.begin());
  return data;
}

static sensor_msgs::NavSatFix decode_nav_sat_fix(const uint8_t* const data,
                                      const size_t& size) {
  const fb::sensor_msgs::NavSatFix* src = flatbuffers::GetRoot<fb::sensor_msgs::NavSatFix>(data);
  sensor_msgs::NavSatFix dst;
  dst.altitude = src->altitude();
  dst.latitude = src->latitude();
  dst.longitude = src->longitude();
  dst.header.frame_id = src->header()->frame_id()->str();
  dst.header.seq = src->header()->seq();
  dst.header.stamp = ros::Time(src->header()->stamp()->secs(), src->header()->stamp()->nsecs());
  dst.position_covariance_type = src->position_covariance_type();
  dst.status.service = src->status()->service();
  dst.status.status = src->status()->status();
  return dst;
}
