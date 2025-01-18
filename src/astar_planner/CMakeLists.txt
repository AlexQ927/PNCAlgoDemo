cmake_minimum_required(VERSION 3.8)
project(astar_planner)

set(CMAKE_BUILD_TYPE "Debug")
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
set(msg_files
  "msg/AstarPath.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES geometry_msgs nav_msgs
)
ament_export_dependencies(rosidl_default_runtime)

# 包含目录
# include_directories(
#   include
#   ${rclcpp_INCLUDE_DIRS}
#   ${geometry_msgs_INCLUDE_DIRS}
#   ${nav_msgs_INCLUDE_DIRS}
# )

add_executable(astar_search_node src/astar_search.cpp)
ament_target_dependencies(
  astar_search_node 
  rclcpp 
  std_msgs 
  nav_msgs 
  geometry_msgs
  rosidl_default_runtime
)

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

install(TARGETS
  astar_search_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY msg
  DESTINATION share/${PROJECT_NAME}/
)

# rosidl_get_typesupport_target(cpp_typesupport_target
#   ${PROJECT_NAME} rosidl_typesupport_cpp)

# target_link_libraries(astar_search_node "${cpp_typesupport_target}")
target_link_libraries(astar_search_node
  ${PROJECT_NAME}__rosidl_typesupport_cpp
)
ament_package()
