cmake_minimum_required(VERSION 2.8.3)
project(your_project_name)

# 添加 ethzasl_apriltag2 子目录
add_subdirectory(src/ethzasl_apriltag2)
add_subdirectory(src/april_detect)

# 添加 test 子目录
add_subdirectory(src/test)

# 设置 ethzasl_apriltag2 的包含路径
include_directories(src/ethzasl_apriltag2/include)

# 设置 april_detect 的包含路径
include_directories(src/april_detect/include)

