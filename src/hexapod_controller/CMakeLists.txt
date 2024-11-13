cmake_minimum_required(VERSION 3.8)
project(hexapod_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(hexapod_interfaces REQUIRED)

include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(inverse_kinematics src/inverse_kinematics.cpp)
ament_target_dependencies(inverse_kinematics rclcpp geometry_msgs std_msgs)

add_executable(body_pose_generator src/body_pose_generator.cpp)
ament_target_dependencies(body_pose_generator rclcpp std_msgs sensor_msgs tf2)

add_executable(tip_pose_generator src/tip_pose_generator.cpp)
ament_target_dependencies(tip_pose_generator rclcpp hexapod_interfaces std_msgs)

add_executable(tip_trajectory_generator src/tip_trajectory_generator.cpp)
ament_target_dependencies(tip_trajectory_generator rclcpp geometry_msgs hexapod_interfaces std_msgs)

install(
  DIRECTORY include
  DESTINATION include
)

install(TARGETS
    inverse_kinematics
    body_pose_generator
    tip_pose_generator
    tip_trajectory_generator
  DESTINATION lib/${PROJECT_NAME}/
)


install(
    DIRECTORY config launch
    DESTINATION share/${PROJECT_NAME}
)


ament_package()
