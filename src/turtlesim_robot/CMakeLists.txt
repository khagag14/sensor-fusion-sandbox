cmake_minimum_required(VERSION 3.8)
project(turtlesim_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(
  include/
  include/turtlesim_robot/
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)
find_package(nav_msgs REQUIRED)
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

add_executable( turtlesim_robot_odometry 
        src/odometry.cpp
)

ament_target_dependencies(turtlesim_robot_odometry
 rclcpp 
 geometry_msgs
 tf2
 std_msgs
 turtlesim
 tf2_geometry_msgs
)

add_executable( turtlesim_robot_positioning
        src/positioning.cpp
)

ament_target_dependencies(turtlesim_robot_positioning
 rclcpp 
 geometry_msgs
 nav_msgs
 tf2
 std_msgs
 turtlesim
 tf2_geometry_msgs
)

install(TARGETS
  turtlesim_robot_odometry
  turtlesim_robot_positioning
  DESTINATION lib/${PROJECT_NAME}
  )
