cmake_minimum_required(VERSION 3.5)
project(rs_to_velodyne_ros2)

#Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(PCL REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(
  ${PCL_INCLUDE_DIRS}
  ${PCL_COMMON_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}  src/rs_to_velodyne_ros2.cpp)
ament_target_dependencies(${PROJECT_NAME} rclcpp sensor_msgs )
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})


install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

ament_package()