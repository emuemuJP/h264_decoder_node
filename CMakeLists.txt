cmake_minimum_required(VERSION 3.5)
project(h264_decoder_node)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0)
pkg_check_modules(GST_APP REQUIRED gstreamer-app-1.0)

add_executable(h264_decoder_node src/h264_decoder_node.cpp)

target_include_directories(h264_decoder_node PRIVATE
  ${GSTREAMER_INCLUDE_DIRS}
  ${GST_APP_INCLUDE_DIRS}
)

ament_target_dependencies(h264_decoder_node rclcpp sensor_msgs)

target_link_libraries(h264_decoder_node
  ${GSTREAMER_LIBRARIES}
  ${GST_APP_LIBRARIES}
)

install(TARGETS h264_decoder_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
