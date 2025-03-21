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

#include <string.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <functional>
#include <mutex>

#include <vanjee_driver/common/wj_common.hpp>
namespace vanjee {
namespace lidar {
#pragma pack(push, 1)
typedef struct {
  uint8_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
} WJTimestampYMD;
typedef struct {
  uint8_t sec[4];
  uint8_t ss[4];

} WJTimestampUTC;
#pragma pack(pop)
#ifdef ENABLE_STAMP_WITH_LOCAL
inline long getTimezone(void) {
  static long tzone = 0;
  static bool tzoneReady = false;
  if (!tzoneReady) {
    tzset();
    tzone = timezone;
    tzoneReady = true;
  }
  return tzone;
}
#endif

inline uint64_t parseTimeUTCWithUs_BigEndian(const WJTimestampUTC *tsUtc) {
  uint64_t sec = 0;
  for (int i = 3; i >= 0; i--) {
    sec <<= 8;
    sec += tsUtc->sec[i];
  }

  uint64_t us = 0;
  for (int i = 3; i >= 0; i--) {
    us <<= 8;
    us += tsUtc->ss[i];
  }

#ifdef ENABLE_STAMP_WITH_LOCAL
  sec -= getTimezone();
#endif

  return (sec * 1000000 + us);
}

/// @brief timestamp UTC+8
inline uint64_t parseTimeUTCWithUs(const WJTimestampUTC *tsUtc) {
  uint64_t sec = 0;
  for (int i = 0; i < 4; i++) {
    sec <<= 8;
    sec += tsUtc->sec[i];
  }

  uint64_t us = 0;
  for (int i = 0; i < 4; i++) {
    us <<= 8;
    us += tsUtc->ss[i];
  }

#ifdef ENABLE_STAMP_WITH_LOCAL
  sec -= getTimezone();
#endif

  return (sec * 1000000 + us);
}

inline void createTimeUTCWithUs(uint64_t us, WJTimestampUTC *tsUtc) {
  uint64_t sec = us / 1000000;
  uint64_t usec = us % 1000000;

  for (int i = 3; i >= 0; i--) {
    tsUtc->sec[i] = sec & 0xFF;
    sec >>= 8;
  }

  for (int i = 3; i >= 0; i--) {
    tsUtc->ss[i] = usec & 0xFF;
    usec >>= 8;
  }
}
inline uint64_t parseTimeYMD(const WJTimestampYMD *tsYmd) {
  std::tm stm;
  memset(&stm, 0, sizeof(stm));

  stm.tm_year = tsYmd->year + (2000 - 1900);
  stm.tm_mon = tsYmd->month - 1;
  stm.tm_mday = tsYmd->day;
  stm.tm_hour = tsYmd->hour;
  stm.tm_min = tsYmd->minute;
  stm.tm_sec = tsYmd->second;
  time_t sec = std::mktime(&stm);

#if 0
  std::cout << "tm_year:" << stm.tm_year 
    << ", tm_mon:" << stm.tm_mon 
    << ", tm_day:" << stm.tm_mday
    << ", tm_hour:" << stm.tm_hour
    << ", tm_min:" << stm.tm_min
    << ", tm_sec:" << stm.tm_sec
    << ", ms:" << ms 
    << ", us:" << us 
    << std::endl;
#endif

#ifdef ENABLE_STAMP_WITH_LOCAL
  sec -= getTimezone();
#endif

  return (sec * 1000000);
}

inline void createTimeYMD(uint64_t usec, WJTimestampYMD *tsYmd) {
  uint64_t us = usec % 1000;
  uint64_t tot_ms = (usec - us) / 1000;

  uint64_t sec = tot_ms / 1000;

#ifdef ENABLE_STAMP_WITH_LOCAL
  sec += getTimezone();
#endif

  time_t t_sec = sec;

  std::tm *stm = localtime(&t_sec);

#if 0
  std::cout << "+ tm_year:" << stm->tm_year 
    << ", tm_mon:" << stm->tm_mon 
    << ", tm_day:" << stm->tm_mday
    << ", tm_hour:" << stm->tm_hour
    << ", tm_min:" << stm->tm_min
    << ", tm_sec:" << stm->tm_sec
    << ", ms:" << ms 
    << ", us:" << us 
    << std::endl;
#endif
  tsYmd->year = stm->tm_year - (2000 - 1900);
  tsYmd->month = stm->tm_mon + 1;
  tsYmd->day = stm->tm_mday;
  tsYmd->hour = stm->tm_hour;
  tsYmd->minute = stm->tm_min;
  tsYmd->second = stm->tm_sec;
}

inline uint64_t getTimeHost(void) {
  std::chrono::system_clock::time_point t = std::chrono::system_clock::now();
  std::chrono::system_clock::duration t_s = t.time_since_epoch();

  std::chrono::duration<uint64_t, std::ratio<1l, 1000000l>> t_us =
      std::chrono::duration_cast<std::chrono::duration<uint64_t, std::ratio<1l, 1000000l>>>(t_s);
  return t_us.count();
}

}  // namespace lidar
}  // namespace vanjee
