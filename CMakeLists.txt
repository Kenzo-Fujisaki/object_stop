cmake_minimum_required(VERSION 3.8)
project(object_stop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O3 -Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_ros_pkgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
)

set(dependencies
  "geometry_msgs"
  "nav_msgs"
  "rclcpp"
  "sensor_msgs"
  "tf2"
)

set(EXEC_NAME "drive_object_stop")

add_executable(${EXEC_NAME} src/drive_object_stop.cpp)
ament_target_dependencies(${EXEC_NAME} ${dependencies})

add_library(obstacle1 SHARED ~/ros2_ws/src/turtlebot3_simulations/turtlebot3_gazebo/models/turtlebot3_dqn_world/obstacle_plugin/obstacle1.cc)
target_link_libraries(obstacle1 ${GAZEBO_LIBRARIES})

add_library(obstacle2 SHARED ~/ros2_ws/src/turtlebot3_simulations/turtlebot3_gazebo/models/turtlebot3_dqn_world/obstacle_plugin/obstacle2.cc)
target_link_libraries(obstacle2 ${GAZEBO_LIBRARIES})

add_library(obstacles SHARED ~/ros2_ws/src/turtlebot3_simulations/turtlebot3_gazebo/models/turtlebot3_dqn_world/obstacle_plugin/obstacles.cc)
target_link_libraries(obstacles ${GAZEBO_LIBRARIES})


install(TARGETS ${EXEC_NAME}
  DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY include/
  DESTINATION include/
)


ament_export_include_directories(include)
ament_export_dependencies(gazebo_ros_pkgs)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(nav_msgs)
ament_export_dependencies(rclcpp)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(tf2)
ament_package()
