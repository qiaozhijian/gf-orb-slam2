export DEPENDENCIES_DIR=/mnt/DATA/SDK
cd ${DEPENDENCIES_DIR}/opencv-3.4.1
mkdir build
cd build
## build opencv without cuda support
# cmake .. -DCMAKE_INSTALL_PREFIX:PATH="/opt/opencv3" -DBUILD_TBB:BOOL="1" -DWITH_TBB:BOOL="1" -DCMAKE_BUILD_TYPE:STRING="Release" -DWITH_OPENMP:BOOL="1"  -DBUILD_opencv_gpu:BOOL="0" -DOPENCV_EXTRA_MODULES_PATH:PATH="/mnt/DATA/SDK/opencv_contrib-3.4.1/modules" -DBUILD_opencv_cudaobjdetect:BOOL="0" -DWITH_CUFFT:BOOL="0" -DBUILD_opencv_cudaimgproc:BOOL="0" -DBUILD_opencv_cudastereo:BOOL="0" -DBUILD_opencv_cudaoptflow:BOOL="0" -DBUILD_opencv_cudabgsegm:BOOL="0" -DBUILD_opencv_cudaarithm:BOOL="0" -DWITH_CUDA:BOOL="0" -DOPENCV_ENABLE_NONFREE:BOOL="1" -DBUILD_opencv_cudacodec:BOOL="0" -DWITH_CUBLAS:BOOL="0" -DBUILD_opencv_cudawarping:BOOL="0" -DBUILD_opencv_cudafilters:BOOL="0" -DCUDA_64_BIT_DEVICE_CODE:BOOL="0" -DBUILD_opencv_cudafeatures2d:BOOL="0" -DBUILD_opencv_cudalegacy:BOOL="0" -DEIGEN_INCLUDE_PATH:PATH="/opt/eigen33/include/eigen3" 
## build opencv with cuda support
cmake .. -DCMAKE_INSTALL_PREFIX:PATH="/opt/opencv3" -DBUILD_TBB:BOOL="1" -DWITH_TBB:BOOL="1" -DCMAKE_BUILD_TYPE:STRING="Release" -DWITH_OPENMP:BOOL="1"  -DBUILD_opencv_gpu:BOOL="1" -DOPENCV_EXTRA_MODULES_PATH:PATH=${DEPENDENCIES_DIR}/opencv_contrib-3.4.1/modules -DBUILD_opencv_cudaobjdetect:BOOL="1" -DWITH_CUFFT:BOOL="1" -DBUILD_opencv_cudaimgproc:BOOL="1" -DBUILD_opencv_cudastereo:BOOL="1" -DBUILD_opencv_cudaoptflow:BOOL="1" -DBUILD_opencv_cudabgsegm:BOOL="1" -DBUILD_opencv_cudaarithm:BOOL="1" -DWITH_CUDA:BOOL="1" -DOPENCV_ENABLE_NONFREE:BOOL="1" -DBUILD_opencv_cudacodec:BOOL="1" -DWITH_CUBLAS:BOOL="1" -DBUILD_opencv_cudawarping:BOOL="1" -DBUILD_opencv_cudafilters:BOOL="1" -DCUDA_64_BIT_DEVICE_CODE:BOOL="1" -DBUILD_opencv_cudafeatures2d:BOOL="1" -DBUILD_opencv_cudalegacy:BOOL="1" -DEIGEN_INCLUDE_PATH:PATH="/opt/eigen33/include/eigen3" 
make -j4
sudo make install

# build Pangolin
sudo apt-get install libglew-dev

cd ${DEPENDENCIES_DIR}
# wget https://github.com/stevenlovegrove/Pangolin/archive/v0.5.tar.gz
#tar xf v0.5.tar.gz
cd ${DEPENDENCIES_DIR}/Pangolin-0.5/
mkdir build
cd build
cmake .. -DCMAKE_INSTALL_PREFIX:PATH="/opt/Pangolin" -DCMAKE_BUILD_TYPE:STRING="Release" -DEIGEN3_INCLUDE_DIR:PATH="/opt/eigen33/include/eigen3" -DLIBREALSENSE_INCLUDE_DIR:PATH="" -DLIBREALSENSE_LIBRARY:FILEPATH="" 
make -j4
sudo make install

# build gtest
cd /usr/src/gtest/
sudo mkdir build
cd build
sudo cmake ..
sudo make -j4
sudo cp libgtest.a ../

# last, install gflags
sudo apt install libgflags-dev
