﻿/*********************************************************************************************************************
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

#include <vanjee_driver/driver/decoder/decoder.hpp>
#include <vanjee_driver/driver/decoder/wlr720/decoder/decoder_vanjee_720.hpp>
#include <vanjee_driver/driver/decoder/wlr720_32/decoder/decoder_vanjee_720_32.hpp>
#include <vanjee_driver/driver/decoder/wlr722/decoder/decoder_vanjee_722.hpp>

namespace vanjee {
namespace lidar {
template <typename T_PointCloud>
class DecoderFactory {
 public:
  static std::shared_ptr<Decoder<T_PointCloud>> createDecoder(LidarType type, const WJDecoderParam &param);
};
template <typename T_PointCloud>
inline std::shared_ptr<Decoder<T_PointCloud>> DecoderFactory<T_PointCloud>::createDecoder(LidarType type, const WJDecoderParam &param) {
  std::shared_ptr<Decoder<T_PointCloud>> ret_ptr;
  switch (type) {
    case LidarType::vanjee_720_16:
      ret_ptr = std::make_shared<DecoderVanjee720<T_PointCloud>>(param);
      break;
    case LidarType::vanjee_720_32:
      ret_ptr = std::make_shared<DecoderVanjee720_32<T_PointCloud>>(param);
      break;
    case LidarType::vanjee_722:
      ret_ptr = std::make_shared<DecoderVanjee722<T_PointCloud>>(param);
      break;

    default:
      exit(-1);
  }
  return ret_ptr;
}

}  // namespace lidar
}  // namespace vanjee
