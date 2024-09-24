cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_CXX_STANDARD 17)  # 设置C++标准为17
project(test)

# Add compiler flags
add_definitions("-fPIC -O3")

# Source files
file(GLOB SOURCE_FILES "*.cpp")

# Executable
add_executable(corner ${SOURCE_FILES})

# Boost libraries
set(BOOST_ROOT "../../3rdparty/boost_install")
find_package(Boost REQUIRED COMPONENTS filesystem)  # 替换your_required_components为实际需要的Boost组件

if(Boost_FOUND)  
  include_directories(${Boost_INCLUDE_DIRS})  # 添加Boost库的include目录  
  target_link_libraries(corner ${Boost_LIBRARIES})  # 链接Boost库
  message(STATUS "Boost library directories: ${Boost_LIBRARY_DIRS}")
endif() 

# Include directories for ethzasl_apriltag2
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../S/include)

# Include directories for april_detect
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../april_detect/include)

#include directories for nlohmann
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../utilities)

# Link with ethzasl_apriltag2 library
target_link_libraries(corner ethzasl_apriltag2 april_detect) 

# 或者如果你想使用变量
# target_link_libraries(corner ${ETHZASL_APRILTAG2_LIB} ${APRIL_DETECT_LIB} )
