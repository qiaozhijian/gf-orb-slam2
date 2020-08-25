/**
* This file is part of GF-ORB-SLAM2.
*
* Copyright (C) 2019 Yipu Zhao <yipu dot zhao at gatech dot edu> 
* (Georgia Institute of Technology)
* For more information see 
* <https://sites.google.com/site/zhaoyipu/good-feature-visual-slam>
*
* GF-ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* GF-ORB-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with GF-ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef UTIL_CUDA_HPP
#define UTIL_CUDA_HPP

//
#include <chrono>
#include <iostream>

#define SET_CLOCK(t0) \
    std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();

#define TIME_DIFF(t1, t0) \
    (std::chrono::duration_cast<std::chrono::duration<double>>((t1) - (t0)).count())

#define PRINT_CLOCK(msg, t1, t0) \
    std::cerr << msg << TIME_DIFF(t1, t0) << endl;

#ifdef USE_NVTX

#include "nvToolsExt.h"

static const uint32_t colors[] = {0x0000ff00, 0x000000ff, 0x00ffff00, 0x00ff00ff, 0x0000ffff, 0x00ff0000, 0x00ffffff};
static const int num_colors = sizeof(colors) / sizeof(uint32_t);

#define PUSH_RANGE(name, cid)                              \
    {                                                      \
        int color_id = cid;                                \
        color_id = color_id % num_colors;                  \
        nvtxEventAttributes_t eventAttrib = {0};           \
        eventAttrib.version = NVTX_VERSION;                \
        eventAttrib.size = NVTX_EVENT_ATTRIB_STRUCT_SIZE;  \
        eventAttrib.colorType = NVTX_COLOR_ARGB;           \
        eventAttrib.color = colors[color_id];              \
        eventAttrib.messageType = NVTX_MESSAGE_TYPE_ASCII; \
        eventAttrib.message.ascii = name;                  \
        nvtxRangePushEx(&eventAttrib);                     \
    }

#define POP_RANGE nvtxRangePop();

#else

#define PUSH_RANGE(name, cid)
#define POP_RANGE

#endif

#endif
