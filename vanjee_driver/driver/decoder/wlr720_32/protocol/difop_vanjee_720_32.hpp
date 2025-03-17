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
#include <memory>
#include <vector>

#include "vanjee_driver/common/super_header.hpp"
#include "vanjee_driver/driver/difop/difop_base.hpp"
#include "vanjee_driver/driver/difop/protocol_abstract.hpp"

namespace vanjee {
namespace lidar {
class DifopVanjee720_32 : public DifopBase {
 public:
  virtual void initGetDifoCtrlDataMapPtr();
};

inline void DifopVanjee720_32::initGetDifoCtrlDataMapPtr() {
  getDifoCtrlData_map_ptr_ = std::make_shared<std::map<uint16, GetDifoCtrlClass>>();

  GetDifoCtrlClass getDifoCtrlData_LdAngleGet(*(std::make_shared<Protocol_LDAngleGet720_32>()->GetRequest()));
  (*getDifoCtrlData_map_ptr_).emplace(CmdRepository720_32::CreateInstance()->sp_ld_angle_get_->GetCmdKey(), getDifoCtrlData_LdAngleGet);

  // GetDifoCtrlClass getDifoCtrlData_ImuLineGet(*(std::make_shared<Protocol_ImuLineGet720_32>()->GetRequest()));
  // (*getDifoCtrlData_map_ptr_).emplace(CmdRepository720_32::CreateInstance()->sp_imu_line_Param_get_->GetCmdKey(), getDifoCtrlData_ImuLineGet);

  // GetDifoCtrlClass getDifoCtrlData_IMUAddGet(*(std::make_shared<Protocol_ImuAddGet720_32>()->GetRequest()));
  // (*getDifoCtrlData_map_ptr_).emplace(CmdRepository720_32::CreateInstance()->sp_imu_add_Param_get_->GetCmdKey(), getDifoCtrlData_IMUAddGet);

  // GetDifoCtrlClass getDifoCtrlData_ImuTempGet(*(std::make_shared<Protocol_ImuTempGet720_32>()->GetRequest()), false, 1000);
  // (*getDifoCtrlData_map_ptr_).emplace(CmdRepository720_32::CreateInstance()->sp_temperature_param_get_->GetCmdKey(), getDifoCtrlData_ImuTempGet);
}
}  // namespace lidar
}  // namespace vanjee
