cmake_minimum_required(VERSION 3.8)
project(hexapod_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

# Uncomment to add more dependencies manually
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    set(ament_cmake_copyright_FOUND TRUE)
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif()

# Install the Python script (keyboard_publisher.py)
install(PROGRAMS
    ${PROJECT_NAME}/keyboard_publisher.py
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
