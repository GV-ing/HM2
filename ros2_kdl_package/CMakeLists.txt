cmake_minimum_required(VERSION 3.8)
project(ros2_kdl_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(urdf REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Punto 1c: generazione interfaccia Action (FollowTrajectory)
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/FollowTrajectory.action"
)

# Necessario per caricare le librerie di typesupport a runtime
ament_export_dependencies(rosidl_default_runtime)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# Nodo principale (Punti 1a/1b): parametri + controllore con spazio nullo
add_executable(ros2_kdl_node src/ros2_kdl_node.cpp src/kdl_robot.cpp src/kdl_planner.cpp src/kdl_control.cpp)
target_include_directories(ros2_kdl_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS})
target_compile_features(ros2_kdl_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(ros2_kdl_node 
  orocos_kdl
  rclcpp
  rclcpp_action
  kdl_parser
  sensor_msgs
  geometry_msgs
  std_msgs)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(ros2_kdl_node "${cpp_typesupport_target}")

install(TARGETS ros2_kdl_node
  DESTINATION lib/${PROJECT_NAME})

add_executable(ros2_kdl_node_example src/ros2_kdl_node_example.cpp)
target_include_directories(ros2_kdl_node_example PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS})
target_compile_features(ros2_kdl_node_example PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(ros2_kdl_node_example 
  orocos_kdl
  rclcpp
)

install(TARGETS ros2_kdl_node_example
  DESTINATION lib/${PROJECT_NAME})

# Client azione di test (Punto 1c)
add_executable(trajectory_action_client src/trajectory_action_client.cpp)
target_include_directories(trajectory_action_client PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS})
target_compile_features(trajectory_action_client PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(trajectory_action_client 
  rclcpp
  rclcpp_action)

rosidl_get_typesupport_target(cpp_typesupport_target_client
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(trajectory_action_client "${cpp_typesupport_target_client}")

install(TARGETS trajectory_action_client
  DESTINATION lib/${PROJECT_NAME})

# Server azione (Punto 1c)
add_executable(trajectory_action_server src/trajectory_action_server.cpp src/kdl_robot.cpp src/kdl_planner.cpp src/kdl_control.cpp)
target_include_directories(trajectory_action_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS})
target_compile_features(trajectory_action_server PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(trajectory_action_server
  orocos_kdl
  rclcpp
  rclcpp_action
  kdl_parser
  sensor_msgs
  geometry_msgs
  std_msgs)

rosidl_get_typesupport_target(cpp_typesupport_target_server
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(trajectory_action_server "${cpp_typesupport_target_server}")

install(TARGETS trajectory_action_server
  DESTINATION lib/${PROJECT_NAME})

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

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
