cmake_minimum_required(VERSION 3.8)
project(a2_rt_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ClosestObstInfo.msg"
  "srv/ChangeThreshold.srv"
  "srv/AverageVelocities.srv"
)

add_executable(threshold_service src/threshold_service.cpp)
ament_target_dependencies(threshold_service rclcpp)
rosidl_target_interfaces(threshold_service
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(avg_vel_service src/avg_vel_service.cpp)
ament_target_dependencies(avg_vel_service rclcpp geometry_msgs)
rosidl_target_interfaces(avg_vel_service
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS threshold_service avg_vel_service
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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
