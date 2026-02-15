cmake_minimum_required(VERSION 3.8)
project(swarm_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# 安装 launch 和 config 文件夹
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()