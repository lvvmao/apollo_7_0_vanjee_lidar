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

#include "vanjee_driver/driver/decoder/wlr720/protocol/difop_vanjee_720.hpp"
#include "vanjee_driver/driver/decoder/wlr720_32/protocol/difop_vanjee_720_32.hpp"
#include "vanjee_driver/driver/decoder/wlr722/protocol/difop_vanjee_722.hpp"
#include "vanjee_driver/driver/difop/difop_base.hpp"
#include "vanjee_driver/driver/difop/protocol_base.hpp"

namespace vanjee {
namespace lidar {
class DifopFactory {
 public:
  static std::shared_ptr<DifopBase> createDifop(WJDriverParam param);
};

inline std::shared_ptr<DifopBase> DifopFactory::createDifop(WJDriverParam param) {
  std::shared_ptr<DifopBase> ret_ptr;
  ProtocolBase pb;

  switch (param.lidar_type) {
    case LidarType::vanjee_720_16:
      ret_ptr = std::make_shared<DifopVanjee720>();
      break;
    case LidarType::vanjee_720_32:
      ret_ptr = std::make_shared<DifopVanjee720_32>();
      break;
    case LidarType::vanjee_722:
      ret_ptr = std::make_shared<DifopVanjee722>();
      break;

    default:
      exit(-1);
  }

  return ret_ptr;
}
}  // namespace lidar
}  // namespace vanjee