cmake_minimum_required(VERSION 3.16)
project(kimera_vio_ros)

# Kimera-VIO core builds with C++17.
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic
                      -Wno-unused-parameter
                      -Wno-deprecated-declarations
                      -Wno-error)
endif()

# ROS 2 dependencies.
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(message_filters REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
# Slice C: pose_graph_tools_msgs is sourced from the ~/hydra_ws/ overlay.
# colcon build picks it up via `source ~/hydra_ws/install/setup.bash` before
# the kimera_ws build, so no clone is required.
find_package(pose_graph_tools_msgs REQUIRED)

# Kimera-VIO core (installed to /opt/kimera-deps in Phase 1).
list(APPEND CMAKE_PREFIX_PATH "/opt/kimera-deps")
find_package(kimera_vio CONFIG REQUIRED)

# Pull in extra deps that kimera_vioConfig.cmake doesn't transitively forward
# all the way (gflags, glog used directly in our wrapper).
find_package(gflags REQUIRED)
find_package(Eigen3 REQUIRED)

# OpenCV via cv_bridge transitively pulls headers, but we use cv::Mat
# directly so request it explicitly.
find_package(OpenCV REQUIRED COMPONENTS core imgproc)

# Locate Glog through Kimera's FindGlog.cmake (installed alongside Config).
list(APPEND CMAKE_MODULE_PATH "/opt/kimera-deps/lib/cmake/kimera_vio")
find_package(Glog REQUIRED)

set(KIMERA_VIO_ROS_LIB_NAME ${PROJECT_NAME})

add_library(${KIMERA_VIO_ROS_LIB_NAME} SHARED
  src/KimeraVioRos.cpp
  src/LcdRegistrationServer.cpp
  src/RosDataProviderInterface.cpp
  src/RosDisplay.cpp
  src/RosLoopClosureVisualizer.cpp
  src/RosOnlineDataProvider.cpp
  src/RosVisualizer.cpp
  src/utils/UtilsRos.cpp
)

target_include_directories(${KIMERA_VIO_ROS_LIB_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

target_link_libraries(${KIMERA_VIO_ROS_LIB_NAME}
  PUBLIC
    kimera_vio
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    gflags
    ${GLOG_LIBRARIES}
)

ament_target_dependencies(${KIMERA_VIO_ROS_LIB_NAME}
  PUBLIC
    rclcpp
    std_srvs
    std_msgs
    nav_msgs
    geometry_msgs
    sensor_msgs
    visualization_msgs
    tf2
    tf2_ros
    cv_bridge
    image_transport
    message_filters
    pcl_msgs
    pcl_conversions
    pose_graph_tools_msgs
)

# Main executable.
add_executable(kimera_vio_ros_node src/KimeraVioRosNode.cpp)
target_link_libraries(kimera_vio_ros_node
  PRIVATE ${KIMERA_VIO_ROS_LIB_NAME})

# RPATH housekeeping: the installed binary depends on libkimera_vio.so (which
# lives in /opt/kimera-deps/lib after Phase 1's `cmake --install`) and on the
# Kimera-VIO build tree as a fallback for dev iteration. Embedding both lets
# `ros2 launch kimera_vio_ros …` work without LD_LIBRARY_PATH manipulation.
# INSTALL_RPATH_USE_LINK_PATH=TRUE additionally appends the link-time paths
# of every transitive .so we link to (covers /opt/kimera-deps/lib for GTSAM,
# DBoW2, KimeraRPGO).
set(_KIMERA_RPATHS
    "/home/maikel/coding/MITSPARK/Kimera-VIO/build"
    "/opt/kimera-deps/lib")
set_target_properties(${KIMERA_VIO_ROS_LIB_NAME} PROPERTIES
    INSTALL_RPATH "${_KIMERA_RPATHS}"
    INSTALL_RPATH_USE_LINK_PATH TRUE
    BUILD_WITH_INSTALL_RPATH FALSE)
set_target_properties(kimera_vio_ros_node PROPERTIES
    INSTALL_RPATH "${_KIMERA_RPATHS}"
    INSTALL_RPATH_USE_LINK_PATH TRUE
    BUILD_WITH_INSTALL_RPATH FALSE)

install(TARGETS ${KIMERA_VIO_ROS_LIB_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS kimera_vio_ros_node
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL)

# Phase 2.5 light: Python visualization helper.
install(PROGRAMS scripts/mesh_to_marker.py
  DESTINATION lib/${PROJECT_NAME}
  OPTIONAL)

ament_export_include_directories(include)
ament_export_libraries(${KIMERA_VIO_ROS_LIB_NAME})
ament_export_dependencies(rclcpp std_srvs std_msgs nav_msgs geometry_msgs
                          sensor_msgs visualization_msgs tf2 tf2_ros cv_bridge
                          image_transport message_filters pcl_msgs
                          pcl_conversions pose_graph_tools_msgs)
ament_package()
