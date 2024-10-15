cmake_minimum_required(VERSION 3.5)
project(dynamic_ground_image)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(pluginlib REQUIRED)

include_directories(include)

set(CMAKE_CXX_STANDARD 14)

add_library(${PROJECT_NAME} SHARED
    src/dynamic_ground_image.cpp
)

ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    sensor_msgs
    cv_bridge
    OpenCV
    rviz_common
    rviz_rendering
    pluginlib
)

install(TARGETS ${PROJECT_NAME}
    LIBRARY DESTINATION lib
)

install(DIRECTORY include/
    DESTINATION include
)

install(FILES plugin_description.xml
    DESTINATION share/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

ament_package()
