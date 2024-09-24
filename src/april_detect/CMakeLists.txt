# CMakeLists.txt in the april_detect directory
set(CMAKE_CXX_STANDARD 17)  # 设置C++标准为17  
project(april_detect)  
  
set(ETHZASL_APRILTAG2_HEAD "../ethzasl_apriltag2/include")  
# 设置头文件目录  
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)  
include_directories(${ETHZASL_APRILTAG2_HEAD})  

# CUDA
set(CUDA_TOOLKIT_ROOT_DIR "/usr/local/cuda-10.2")
find_package(CUDA 10.2 REQUIRED)
find_package(CUDAToolkit REQUIRED)
find_library(CUVID_LIB nvcuvid)
find_library(NVENCODEAPI_LIB nvidia-encode)
message(STATUS ${CUVID_LIB})
message(STATUS ${NVENCODEAPI_LIB})

# OpenCV
set(OpenCV_DIR "../../3rdparty/opencv_install/lib/cmake/opencv4")
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
message(STATUS "OpenCV path is ${OpenCV_INCLUDE_DIRS}")

# Eigen3
set(Eigen3_DIR "../../3rdparty/eigen_install/share/eigen3/cmake")
find_package(Eigen3 3.3.7 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
message(STATUS "Eigen3 path is ${EIGEN3_INCLUDE_DIR}")

# 添加头文件  
set(APRIL_DETECT_HEADERS  
    include/cameras/GridCalibrationTargetAprilgrid.hpp  
)  
  
# Source files  
file(GLOB SOURCE_FILES "src/*.cpp")  

# 添加库的构建  
add_library(${PROJECT_NAME} STATIC ${SOURCE_FILES})  

 
target_link_libraries(${PROJECT_NAME} ${CMAKE_THREAD_LIBS_INIT} ${OpenCV_LIBS} ethzasl_apriltag2)
target_include_directories(${PROJECT_NAME} PUBLIC ../ethzasl_apriltag2/include)