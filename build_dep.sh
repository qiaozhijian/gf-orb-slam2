export DEPENDENCIES_DIR=/mnt/DATA/SDK

mkdir -p ${DEPENDENCIES_DIR}
cd ${DEPENDENCIES_DIR}

# assuming ros-indigo has been configured properly, and the gcc is the standard 4.8.5 for ubuntu 14.04.5 LTS
# also, make sure no gfortran is installed; otherwise openblas will build lapack and blas, which are leading to worse performance of subset selection than lapack-dev
# sudo apt-get remove libgfortran-4.*-dev

# make sure no openblas being installed
# sudo apt-get remove libopenblas-base

# build openblas with single-thread
# wget https://sourceforge.net/projects/openblas/files/v0.3.5/OpenBLAS%200.3.5%20version.zip
# unzip OpenBLAS\ 0.3.5\ version.zip
cd ${DEPENDENCIES_DIR}/xianyi-OpenBLAS-eebc189/
make USE_THREAD=0 
sudo make PREFIX=/opt/OpenBLAS install

# build armadillo
cd ${DEPENDENCIES_DIR}
# wget https://sourceforge.net/projects/arma/files/armadillo-9.200.7.tar.xz
#tar xf armadillo-9.200.7.tar.xz
cd ${DEPENDENCIES_DIR}/armadillo-9.200.7/
mkdir build
cd build
cmake .. -DCMAKE_INSTALL_PREFIX:PATH="/opt/armadillo" -DCMAKE_INSTALL_LIBDIR:PATH="lib" -DCMAKE_BUILD_TYPE:STRING="Release" -Dopenblas_LIBRARY:FILEPATH="/opt/OpenBLAS/lib/libopenblas.so" -Dopenblasp_LIBRARY:FILEPATH="/opt/OpenBLAS/lib/libopenblas.so" -Dopenblaso_LIBRARY:FILEPATH="/opt/OpenBLAS/lib/libopenblas.so" -DLAPACK_LIBRARY:FILEPATH="/opt/OpenBLAS/lib/libopenblas.so" 
make -j4
sudo make install

# build eigen
cd ${DEPENDENCIES_DIR}
# wget http://bitbucket.org/eigen/eigen/get/3.3.3.tar.bz2
#tar xf 3.3.3.tar.bz2 eigen-3.3.3
cd ${DEPENDENCIES_DIR}/eigen-3.3.3/
mkdir build
cd build
cmake .. -DCMAKE_INSTALL_PREFIX:PATH="/opt/eigen33" -DCMAKE_BUILD_TYPE:STRING="Release" # -DEIGEN_TEST_CXX11:BOOL="1" 
make -j4
sudo make install

# # build opencv2
# cd ${DEPENDENCIES_DIR}
# wget https://github.com/opencv/opencv/archive/2.4.13.6.tar.gz
# tar xf 2.4.13.6.tar.gz
# cd ${DEPENDENCIES_DIR}/opencv-2.4.13.6
# mkdir build
# cd build
# cmake .. -DCMAKE_INSTALL_PREFIX:PATH="/opt/opencv2" -DBUILD_TBB:BOOL="1" -DWITH_CUFFT:BOOL="0" -DWITH_TBB:BOOL="1" -DWITH_CUDA:BOOL="0" -DCUDA_HOST_COMPILATION_CPP:BOOL="0" -DCMAKE_BUILD_TYPE:STRING="Release" -DWITH_OPENMP:BOOL="1" -DCUDA_PROPAGATE_HOST_FLAGS:BOOL="0" -DCUDA_64_BIT_DEVICE_CODE:BOOL="0" -DCUDA_ATTACH_VS_BUILD_RULE_TO_CUDA_FILE:BOOL="0" -DBUILD_opencv_gpu:BOOL="0" 
# make -j
# sudo make install

# build opencv 3.4
cd ${DEPENDENCIES_DIR}
# wget https://github.com/opencv/opencv/archive/3.4.1.tar.gz
#tar xf 3.4.1.tar.gz
# wget https://github.com/opencv/opencv_contrib/archive/3.4.1.tar.gz
#tar xf 3.4.1.tar.gz.1
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
