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

#pragma once
#ifndef __ORB_HPP__
#define __ORB_HPP__

#include <vector>
#include <opencv2/core/cuda.hpp>
#include <opencv2/core/cuda_stream_accessor.hpp>
#include <cuda_runtime.h>
#include <cuda/Cuda.hpp>

namespace ORB_SLAM2 { namespace cuda {
  using namespace std;
  using namespace cv;
  using namespace cv::cuda;

  class GpuOrb {
    unsigned int maxKeypoints;
    KeyPoint * keypoints;
    GpuMat descriptors;
    GpuMat desc;
    cudaStream_t stream;
    Stream cvStream;
  public:
    GpuOrb(int maxKeypoints = 10000);
    ~GpuOrb();

    void launch_async(InputArray _image, const KeyPoint * _keypoints, const int npoints);
    void join(Mat &_descriptors);

    static void loadPattern(const Point * _pattern);
  };
} }
#endif
