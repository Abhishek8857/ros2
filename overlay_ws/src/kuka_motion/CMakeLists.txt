cmake_minimum_required(VERSION 3.8)
project(kuka_motion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(kuka_motion_plan_action REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

  # CLIENT
add_library(action_client SHARED
  src/kuka_motion_client_TCS.cpp)
target_include_directories(action_client PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(action_client
  PRIVATE "KUKA_MOTION_BUILDING_DLL")
ament_target_dependencies(action_client
  "kuka_motion_plan_action"
  "geometry_msgs"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components")
rclcpp_components_register_node(action_client PLUGIN "kuka_motion::MotionPlanClient" EXECUTABLE kuka_motion_client_TCS)
install(TARGETS
  action_client
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

  # SERVER
add_library(action_server SHARED
  src/kuka_motion.cpp)
target_include_directories(action_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(action_server
  PRIVATE "KUKA_MOTION_BUILDING_DLL")
ament_target_dependencies(action_server
  "kuka_motion_plan_action"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "moveit_ros_planning_interface")
rclcpp_components_register_node(action_server PLUGIN "kuka_motion::MotionPlanServer" EXECUTABLE kuka_motion)
install(TARGETS
  action_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

add_executable(pose_vector_publisher_node src/pose_vector_publisher.cpp)
  ament_target_dependencies(
    pose_vector_publisher_node
    "rclcpp"
    "geometry_msgs"
  )

add_executable(motion_state_generator src/motion_state_generator.cpp)
  ament_target_dependencies(
    motion_state_generator 
    "rclcpp" 
    "geometry_msgs" 
    "kuka_motion_plan_action"
  )

# add_executable(endeffector src/kuka_motion_plan.cpp)
# ament_target_dependencies(
#   endeffector
#   "rclcpp"
#   "moveit_ros_planning_interface"
#   "srdfdom"
# )
install(TARGETS pose_vector_publisher_node 
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS motion_state_generator DESTINATION lib/${PROJECT_NAME})

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
