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
#pragma once
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "modules/drivers/lidar/proto/config.pb.h"
#include "modules/drivers/lidar/proto/vanjee.pb.h"
#include "modules/drivers/lidar/proto/vanjee_config.pb.h"
#include "modules/drivers/proto/pointcloud.pb.h"

#include "cyber/cyber.h"
#include "modules/drivers/lidar/common/driver_factory/driver_base.h"


#include <vanjee_driver/api/lidar_driver.hpp>
#include <vanjee_driver/driver/driver_param.hpp>
#include <vanjee_driver/msg/packet.hpp>
#include <vanjee_driver/msg/point_cloud_msg.hpp>

namespace apollo {
namespace drivers {
namespace vanjee {

typedef ::vanjee::lidar::PointXYZIRT PointT;
typedef ::vanjee::lidar::PointCloudT<PointT> PointCloudMsg;
using ::vanjee::lidar::InputType;

static constexpr float kSecondToNanoFactor = 1e9f;

class VanjeeDriver : public lidar::LidarDriver {
 public:
  VanjeeDriver(const std::shared_ptr<cyber::Node> &node,
                  const ::apollo::drivers::lidar::config &config)
      : node_(node), conf_(config.vanjee()) {}
  VanjeeDriver(const std::shared_ptr<cyber::Node> &node, const Config &conf)
      : node_(node), conf_(conf) {}
  ~VanjeeDriver() { stop(); }
  bool Init() override;
 private:
  std::shared_ptr<cyber::Node> node_ = nullptr;
  Config conf_;
  std::shared_ptr<cyber::Writer<VanjeeScan>> scan_writer_ = nullptr;
  std::shared_ptr<cyber::Writer<PointCloud>> pointcloud_writer_ = nullptr;

 private:
  void VanjeePacketCallback(const ::vanjee::lidar::Packet& lidar_packet);
  void VanjeeCloudPutCallback(std::shared_ptr<PointCloudMsg> vanjee_cloud);
  std::shared_ptr<PointCloudMsg> VanjeeCloudAllocateCallback(); 
  void ProcessCloud();
  void prepareLidarScanMsg(std::shared_ptr<VanjeeScan> msg);
  void preparePointsMsg(std::shared_ptr<PointCloud> msg);
  void stop() {
    thread_flag_.store(false);
    driver_ptr_->stop();
    process_cloud_thread_.join();
  }

 private:
   std::atomic<bool> thread_flag_ = {true};
   std::thread process_cloud_thread_;
   uint32_t scan_seq_;
   uint32_t points_seq_;
   std::shared_ptr<PointCloud> point_cloud_ptr_;
   std::shared_ptr<VanjeeScan> scan_ptr_;
   ::vanjee::lidar::SyncQueue<std::shared_ptr<PointCloudMsg>> cloud_queue_;
   ::vanjee::lidar::SyncQueue<std::shared_ptr<PointCloudMsg>> free_cloud_queue_;
   std::shared_ptr<::vanjee::lidar::LidarDriver<PointCloudMsg>> driver_ptr_;
};
}  // namespace vanjee
}  // namespace drivers
}  // namespace apollo
