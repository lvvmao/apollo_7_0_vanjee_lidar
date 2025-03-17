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
#include <list>
#include <memory>
#include <string>
#include <thread>

#include "cyber/cyber.h"

#include "modules/drivers/lidar/vanjee/driver/driver.h"

namespace apollo {
namespace drivers {
namespace vanjee {

using apollo::cyber::Component;

class VanjeeComponent : public Component<> {
 public:
  ~VanjeeComponent() {}
  bool Init() override {
    if (!GetProtoConfig(&conf_)) {
      AERROR << "load config error, file:" << config_file_path_;
      return false;
    }
    driver_.reset(new VanjeeDriver(node_, conf_));
    if (!driver_->Init()) {
      AERROR << "driver init error";
      return false;
    }
    return true;
  }


 private:  
  std::shared_ptr<VanjeeDriver> driver_;
  Config conf_;
};

CYBER_REGISTER_COMPONENT(VanjeeComponent)

}  // namespace vanjee
}  // namespace drivers
}  // namespace apollo
