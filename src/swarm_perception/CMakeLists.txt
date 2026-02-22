cmake_minimum_required(VERSION 3.8)
project(swarm_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 寻找 ROS 2 依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

# 寻找 OpenCV 库 (必须!)
find_package(OpenCV REQUIRED)

# 编译我们的节点
add_executable(qr_detector_node src/qr_detector_node.cpp)
target_include_directories(qr_detector_node PUBLIC ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(qr_detector_node rclcpp sensor_msgs std_msgs cv_bridge)
target_link_libraries(qr_detector_node ${OpenCV_LIBRARIES})

# 安装规则
install(TARGETS
  qr_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()