cmake_minimum_required(VERSION 3.8)
project(clik1_node_pkg)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set per debuggare
set(CMAKE_BUILD_TYPE Debug)

set(CONTROLLER_INCLUDE_DEPENDS
  pinocchio
  rclcpp
  trajectory_msgs
  tf2_ros
  tf2_geometry_msgs
  Eigen3
  pluginlib
  rcpputils
  px4_msgs
  interbotix_xs_msgs
  px4_ros_com
  ament_index_cpp
  visualization_msgs
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Trova esplicitamente i componenti di Python per aiutare le dipendenze a valle (Pinocchio)
find_package(Python 3.10 COMPONENTS Development.Module REQUIRED)

foreach(Dependency IN ITEMS ${CONTROLLER_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_executable(clik_uam_node src/clik_uam_node.cpp)
# Specifico quali librerie importate servono perchè il nodo funzioni 
ament_target_dependencies(clik_uam_node
  rclcpp
  tf2_ros
  tf2_geometry_msgs
  pinocchio
  interbotix_xs_msgs
  px4_ros_com
  px4_msgs
  ament_index_cpp
  visualization_msgs
)

target_include_directories(clik_uam_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Collega esplicitamente la libreria Eigen (FONDAMENTALE)
target_link_libraries(clik_uam_node Eigen3::Eigen)

install(TARGETS
  clik_uam_node
  DESTINATION lib/${PROJECT_NAME}
)

# # Install headers
# install(DIRECTORY include/
#   DESTINATION include)
include_directories(include)

# Install launch, config, and model files
install(DIRECTORY
  launch
  config
  model
  scripts
  DESTINATION share/${PROJECT_NAME}
)

# Installa gli script Python
install(PROGRAMS
  scripts/send_sleep_pose.py
  scripts/check_pinocchio.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

# Aggiunta del nuovo nodo rviz_marker_node
add_executable(rviz_marker_node src/rviz_marker_node.cpp)
# Aggiunta delle dipendenze per il nodo rviz_marker_node
ament_target_dependencies(rviz_marker_node 
  rclcpp 
  visualization_msgs 
  geometry_msgs 
  px4_msgs 
  tf2_ros
  Eigen3)
install(TARGETS rviz_marker_node DESTINATION lib/${PROJECT_NAME})

# Aggiunta del nuovo nodo world_to_base_link_broadcaster
add_executable(world_to_base_link_broadcaster src/world_to_base_link_broadcaster.cpp)
ament_target_dependencies(world_to_base_link_broadcaster rclcpp geometry_msgs px4_msgs tf2_ros Eigen3)
install(TARGETS world_to_base_link_broadcaster DESTINATION lib/${PROJECT_NAME})

ament_package()
