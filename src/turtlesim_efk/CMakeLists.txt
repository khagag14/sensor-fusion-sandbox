cmake_minimum_required(VERSION 3.8)
project(turtlesim_efk)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(
  include/

)

include_directories(SYSTEM thirdparty/sophus)

# find dependencies
# find_package(ament_cmake REQUIRED)
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)

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

add_executable( run_ekf_node 
        src/run_ekf.cpp
)

ament_target_dependencies(run_ekf_node
 rclcpp 
 geometry_msgs
 tf2
 std_msgs
 turtlesim
 tf2_geometry_msgs
 nav_msgs

)

install(TARGETS
  run_ekf_node
  DESTINATION lib/${PROJECT_NAME}
  )