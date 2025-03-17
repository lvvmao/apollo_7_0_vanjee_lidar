/*********************************************************************************************************************
Copyright (c) 2023 Vanjee
All rights reserved

By downloading, copying, installing or using the software you agree to this
license. If you do not agree to this license, do not download, install, copy or
use the software.

License Agreement
For Vanjee LiDAR SDK Library
(3-clause BSD License)

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

3. Neither the names of the Vanjee, nor Suteng Innovation Technology, nor the
names of other contributors may be used to endorse or promote products derived
from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*********************************************************************************************************************/

#pragma once

#include "vanjee_driver/common/super_header.hpp"
#include "vanjee_driver/driver/difop/params_abstract.hpp"

namespace vanjee {
namespace lidar {
class Params_ImuPacketGet722 : public ParamsAbstract {
 public:
  uint8_t main_version_;
  uint8_t sub_version_;
  uint32_t imu_sec_;
  uint32_t imu_nsec_;
  uint8_t remain1_[4];
  int32_t imu_linear_acce_x_;
  int32_t imu_linear_acce_y_;
  int32_t imu_linear_acce_z_;
  int32_t imu_angle_voc_x_;
  int32_t imu_angle_voc_y_;
  int32_t imu_angle_voc_z_;
  uint8_t remain2_[4];

 public:
  virtual std::shared_ptr<std::vector<uint8>> GetBytes() {
    std::shared_ptr<std::vector<uint8>> buf = std::make_shared<std::vector<uint8>>();
    return nullptr;
  }

  virtual void Load(ProtocolBase& protocol) {
    auto buf = protocol.Content.data();
    int index = 0;
    main_version_ = *(buf + index++);
    sub_version_ = *(buf + index++);
    memcpy(&imu_sec_, buf + index, sizeof(int32_t));
    index += 4;
    memcpy(&imu_nsec_, buf + index, sizeof(int32_t));
    index += 4;
    for (int i = 0; i < sizeof(remain1_); i++) {
      remain1_[i] = *(buf + index++);
    }
    memcpy(&imu_linear_acce_x_, buf + index, sizeof(int32_t));
    index += 4;
    memcpy(&imu_linear_acce_y_, buf + index, sizeof(int32_t));
    index += 4;
    memcpy(&imu_linear_acce_z_, buf + index, sizeof(int32_t));
    index += 4;
    memcpy(&imu_angle_voc_x_, buf + index, sizeof(int32_t));
    index += 4;
    memcpy(&imu_angle_voc_y_, buf + index, sizeof(int32_t));
    index += 4;
    memcpy(&imu_angle_voc_z_, buf + index, sizeof(int32_t));
    index += 4;
    for (int i = 0; i < sizeof(remain2_); i++) {
      remain2_[i] = *(buf + index++);
    }
  }
};
}  // namespace lidar
}  // namespace vanjee