cmake_minimum_required(VERSION 3.8)
project(swarm_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# 安装文件夹到 share 目录
install(
  DIRECTORY urdf worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()