cmake_minimum_required(VERSION 3.8)
project(hexapod_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)

# Uncomment to add more dependencies manually
# find_package(<dependency> REQUIRED)

add_executable(float_32_republisher src/float_32_republisher.cpp)
ament_target_dependencies(float_32_republisher rclcpp std_msgs)

install(TARGETS
    float_32_republisher
  DESTINATION lib/${PROJECT_NAME}/
)


ament_package()
