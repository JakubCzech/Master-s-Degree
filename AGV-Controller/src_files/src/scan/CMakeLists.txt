cmake_minimum_required(VERSION 3.8)
project(scan)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pointcloud_to_laserscan REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
include_directories(launch)
include_directories(include)

add_executable(scan_node src/scan_node.cpp)
ament_target_dependencies(scan_node "rclcpp" "sensor_msgs" "pcl_conversions" "pointcloud_to_laserscan" "std_msgs")
target_link_libraries(scan_node ${PCL_LIBRARIES})

target_compile_features(scan_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS scan_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
