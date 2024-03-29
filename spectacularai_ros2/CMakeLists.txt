cmake_minimum_required(VERSION 3.8)

project(spectacularai_ros2)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -O2")
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

set(PLUGIN_SRC
  src/ros2_plugin.cpp)

find_package(spectacularAI REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(depthai_ros_msgs REQUIRED) # Depth AI features

include_directories(include)
add_library(${PROJECT_NAME} SHARED ${PLUGIN_SRC})

# Dynamic dependencies of the static library SDK variant
set(SAI_STATIC_MANUAL_LINK_LIBS z)
target_link_libraries(${PROJECT_NAME} PRIVATE spectacularAI::spectacularAI ${SAI_STATIC_MANUAL_LINK_LIBS})
# Strip the output executable
set_target_properties(${PROJECT_NAME} PROPERTIES LINK_FLAGS -s)

rclcpp_components_register_nodes(${PROJECT_NAME} "spectacularAI::ros2::Node")

# ROS dependencies
ament_target_dependencies(${PROJECT_NAME} PUBLIC spectacularAI rclcpp rclcpp_components sensor_msgs nav_msgs tf2_geometry_msgs image_transport message_filters tf2 tf2_ros depthai_ros_msgs)

# Include dynamic spectacularAI SDK if required
get_target_property(SAI_IMPORTED_LOCATION spectacularAI::spectacularAI IMPORTED_LOCATION_RELEASE)
if (NOT(SAI_IMPORTED_LOCATION MATCHES "\\.a$"))
  message("Dynamic library, including it in install directory")
  install(IMPORTED_RUNTIME_ARTIFACTS spectacularAI::spectacularAI)
endif()

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include/)
install(EXPORT ${PROJECT_NAME}Targets DESTINATION share/${PROJECT_NAME}/cmake)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies})

ament_package()
