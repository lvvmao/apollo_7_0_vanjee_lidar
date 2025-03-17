/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#include "modules/drivers/lidar/vanjee/driver/driver.h"

namespace apollo {
namespace drivers {
namespace vanjee {
using apollo::cyber::Node;
using apollo::cyber::Writer;
using apollo::drivers::PointCloud;
using apollo::drivers::PointXYZIT;
using apollo::drivers::vanjee::VanjeeScan;
bool VanjeeDriver::Init() {
  if (node_ == nullptr) {
    AERROR << "node is nullptr";
    return false;
  }
  
  scan_writer_ = node_->CreateWriter<VanjeeScan>(conf_.scan_channel());
  pointcloud_writer_ =
      node_->CreateWriter<PointCloud>(conf_.pointcloud_channel());
  if (scan_writer_ == nullptr) {
    AERROR << "writer:" << conf_.scan_channel()
           << " create error, check cyber is inited.";
    return false;
  }

  driver_ptr_ = std::make_shared<::vanjee::lidar::LidarDriver<PointCloudMsg>>();
  ::vanjee::lidar::WJDecoderParam decoder_param;
  decoder_param.publish_mode = conf_.publish_mode();
  decoder_param.min_distance = conf_.min_distance();
  decoder_param.max_distance = conf_.max_distance();
  decoder_param.start_angle = conf_.start_angle();
  decoder_param.end_angle = conf_.end_angle();
  decoder_param.hide_points_range = conf_.point_cloud_masking();
  decoder_param.use_lidar_clock = conf_.use_lidar_clock();
  decoder_param.ts_first_point = conf_.ts_first_point();
  decoder_param.use_offset_timestamp = conf_.use_offset_timestamp();
  decoder_param.dense_points = conf_.dense_points();
  decoder_param.wait_for_difop = conf_.wait_for_difop();
  decoder_param.config_from_file = conf_.config_from_file();
  decoder_param.angle_path_hor = conf_.angle_path_hor();
  decoder_param.angle_path_ver = conf_.angle_path_ver();

  ::vanjee::lidar::WJInputParam input_param;
  input_param.connect_type = conf_.connect_type();
  input_param.host_msop_port = conf_.host_msop_port();
  input_param.lidar_msop_port = conf_.lidar_msop_port();
  input_param.host_address = conf_.host_address();
  input_param.lidar_address = conf_.lidar_address();

  ::vanjee::lidar::WJDriverParam driver_param;

  driver_param.input_param = input_param;
  driver_param.decoder_param = decoder_param;
  
  driver_param.lidar_type = ::vanjee::lidar::strToLidarType(conf_.model());
  driver_param.input_type = InputType::ONLINE_LIDAR;
  driver_ptr_->regPacketCallback(
      std::bind(&VanjeeDriver::VanjeePacketCallback, this,
                std::placeholders::_1));
                driver_ptr_->regPointCloudCallback(
    std::bind(&VanjeeDriver::VanjeeCloudAllocateCallback, this),
    std::bind(&VanjeeDriver::VanjeeCloudPutCallback, this,
              std::placeholders::_1));
  driver_ptr_->regExceptionCallback([](const ::vanjee::lidar::Error& code) {
      WJ_WARNING << code.toString() << WJ_REND;
  });

  driver_param.print();

  point_cloud_ptr_.reset(new PointCloud);
  scan_ptr_.reset(new VanjeeScan);

  if (!driver_ptr_->init(driver_param)) {
      AERROR << "vanjee Driver init failed";
      return false;
  }

  if (!driver_ptr_->start()) {
      AERROR << "vanjee Driver start failed";
      return false;
  }
  process_cloud_thread_ = std::thread(&VanjeeDriver::ProcessCloud, this);
  return true;
}


void VanjeeDriver::VanjeePacketCallback(
    const ::vanjee::lidar::Packet& lidar_packet) {
  std::shared_ptr<vanjee::VanjeeScanPacket> scan_packet =
      std::make_shared<vanjee::VanjeeScanPacket>();

  if(lidar_packet.is_frame_begin == true){
    std::shared_ptr<VanjeeScan> raw_scan = scan_ptr_;
    prepareLidarScanMsg(raw_scan);
    scan_writer_->Write(raw_scan);
    scan_ptr_.reset(new VanjeeScan);
  }

  scan_ptr_->add_firing_pkts()->set_data(lidar_packet.buf_.data(), lidar_packet.buf_.size());
}


std::shared_ptr<PointCloudMsg>
VanjeeDriver::VanjeeCloudAllocateCallback() {
  std::shared_ptr<PointCloudMsg> point_cloud = free_cloud_queue_.pop();
  if(point_cloud.get() != NULL){
    return point_cloud; 
  }
  return std::make_shared<PointCloudMsg>();
}

void VanjeeDriver::VanjeeCloudPutCallback(
    std::shared_ptr<PointCloudMsg> vanjee_cloud) {
  cloud_queue_.push(vanjee_cloud);
}

void VanjeeDriver::ProcessCloud() {
    while (thread_flag_) {
      std::shared_ptr<PointCloudMsg> msg = cloud_queue_.popWait(1000);
      if (msg.get() == NULL) {
        continue;
      }
  
      std::shared_ptr<PointCloud> raw_cloud = point_cloud_ptr_;
      for (auto p : msg->points) {
        PointXYZIT* point = raw_cloud->add_point();
        point->set_x(p.x);
        point->set_y(p.y);
        point->set_z(p.z);
        point->set_intensity(uint32_t(p.intensity));
        point->set_timestamp(double(p.timestamp)); 
      }

      raw_cloud->set_height(msg->height);
      raw_cloud->set_width(msg->width);
      raw_cloud->set_is_dense(msg->is_dense);
      raw_cloud->set_measurement_time(msg->timestamp);
      raw_cloud->mutable_header()->set_timestamp_sec(msg->timestamp);
      raw_cloud->mutable_header()->set_lidar_timestamp(msg->timestamp * kSecondToNanoFactor);
      this->preparePointsMsg(raw_cloud);
      if (raw_cloud->point_size() != 0) {
        pointcloud_writer_->Write(raw_cloud);
      }

      point_cloud_ptr_.reset(new PointCloud);

      free_cloud_queue_.push(msg);
    }
  }

  void VanjeeDriver::prepareLidarScanMsg(std::shared_ptr<VanjeeScan> msg) {
    msg->mutable_header()->set_sequence_num(++scan_seq_);
    msg->mutable_header()->set_frame_id(conf_.frame_id());
    msg->mutable_header()->set_timestamp_sec(cyber::Time().Now().ToSecond());
    msg->set_model(conf_.model());
  }

  void VanjeeDriver::preparePointsMsg(std::shared_ptr<PointCloud> msg) {
    msg->mutable_header()->set_sequence_num(++points_seq_);
    msg->mutable_header()->set_frame_id(conf_.frame_id());
  }
}  // namespace robosense
}  // namespace drivers
}  // namespace apollo
