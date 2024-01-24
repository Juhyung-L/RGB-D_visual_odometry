cmake_minimum_required(VERSION 3.8)
project(visual_odometry)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_BUILD_TYPE Release)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(PCL 1.3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenCV REQUIRED)
find_package(message_filters REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

set(dependencies
  rclcpp
  geometry_msgs
  visualization_msgs
  sensor_msgs
  message_filters
  cv_bridge
  image_transport
  pcl_conversions
  tf2_ros
)

include_directories(
  include/
  ${PCL_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(main src/main.cpp src/visual_odom_core.cpp)
ament_target_dependencies(main ${dependencies})
target_link_libraries(main ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

INSTALL(TARGETS
  main
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
