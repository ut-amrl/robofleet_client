cmake_minimum_required(VERSION 3.1.0)
project(client)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Qt setup (needs to precede executable)
set(CMAKE_AUTOMOC on)
set(CMAKE_AUTORCC on)
set(CMAKE_AUTOUIC on)
set(CMAKE_INCLUDE_CURRENT_DIR on)
find_package(Qt5 COMPONENTS Core WebSockets REQUIRED)

add_executable(client)
add_subdirectory(src)
include_directories(client include)
target_link_libraries(client Qt5::Core Qt5::WebSockets) # also adds includes

# find CMake packages for installed ROS packages
# this can be done for any catkin-based package
# https://github.com/gerkey/ros1_external_use/blob/master/ros1_msgs
find_package(roscpp REQUIRED)
include_directories(${roscpp_INCLUDE_DIRS})
target_link_libraries(client ${roscpp_LIBRARIES})

find_package(std_msgs REQUIRED)
include_directories(${std_msgs_INCLUDE_DIRS})
target_link_libraries(client ${std_msgs_LIBRARIES})

# build amrl_msgs
add_custom_target(amrl_msgs
  COMMAND ${CMAKE_COMMAND} -E env "ROS_PACKAGE_PATH=${CMAKE_CURRENT_SOURCE_DIR}/amrl_msgs:$ENV{ROS_PACKAGE_PATH}" make
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/amrl_msgs
)
add_dependencies(client amrl_msgs)
include_directories(client amrl_msgs/msg_gen/cpp/include)
