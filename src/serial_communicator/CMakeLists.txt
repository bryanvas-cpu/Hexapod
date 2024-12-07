cmake_minimum_required(VERSION 3.8)
project(serial_communicator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(std_msgs REQUIRED)

pkg_check_modules(SERIAL libserial)

add_executable(serial_communicator src/serial_communicator.cpp)
ament_target_dependencies(serial_communicator rclcpp std_msgs)

target_include_directories(serial_communicator PUBLIC "${SERIAL_INCLUDE_DIRS}")
target_link_libraries(serial_communicator "${SERIAL_LDFLAGS}")

install(TARGETS
    serial_communicator
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    DESTINATION lib/${PROJECT_NAME}
)



ament_package()
