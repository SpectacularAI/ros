cmake_minimum_required(VERSION 3.8)

project(spectacularai_ros2)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -O2")
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

# add the automatically determined parts of the RPATH
# which point to directories outside the build tree to the install RPATH
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# the RPATH to be used when installing, but only if it's not a system directory
list(FIND CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES "${CMAKE_INSTALL_PREFIX}/lib" isSystemDir)
if("${isSystemDir}" STREQUAL "-1")
    set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
endif("${isSystemDir}" STREQUAL "-1")

set(PLUGIN_SRC
  src/ros2_main.cpp
  src/ros2_plugin.cpp)

find_package(spectacularAI REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(depthai_ros_msgs REQUIRED) # Depth AI features

add_executable(vislam ${PLUGIN_SRC})

# Dynamic dependencies of the static library SDK variant
set(SAI_STATIC_MANUAL_LINK_LIBS z)
target_link_libraries(vislam spectacularAI::spectacularAI ${SAI_STATIC_MANUAL_LINK_LIBS})
# Strip the output executable
set_target_properties(vislam PROPERTIES LINK_FLAGS -s)

# ROS dependencies
ament_target_dependencies(vislam rclcpp sensor_msgs nav_msgs tf2_geometry_msgs image_transport message_filters tf2 tf2_ros depthai_ros_msgs)

install(
  TARGETS vislam DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include/${PROJECT_NAME}
)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
ament_package()
