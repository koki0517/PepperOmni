cmake_minimum_required(VERSION 3.8)
project(pepper_omni)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(uec_utils REQUIRED)
find_package(sensor_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(dxl_omni src/dxl_omni.cpp)
target_include_directories(dxl_omni PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(dxl_omni PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(dxl_omni rclcpp dynamixel_sdk geometry_msgs std_msgs uec_utils)

add_executable(joy_pub src/joy_pub.cpp)
target_include_directories(joy_pub PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(joy_pub PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(joy_pub rclcpp geometry_msgs sensor_msgs std_msgs uec_utils)

install(TARGETS joy_pub
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS dxl_omni
  DESTINATION lib/${PROJECT_NAME})

# Install launch files so they are available under share/<package>/launch
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

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

ament_package()
