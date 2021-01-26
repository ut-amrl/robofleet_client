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
list(APPEND Libs Qt5::Core Qt5::WebSockets)

add_executable(client src/main.cpp)
add_executable(test_client test/main.cpp)

add_subdirectory(src)
include_directories(include)

# find CMake packages for installed ROS packages
# this can be done for any catkin-based package
# https://github.com/gerkey/ros1_external_use/blob/master/ros1_msgs
find_package(roscpp REQUIRED)
include_directories(${roscpp_INCLUDE_DIRS})
list(APPEND Libs ${roscpp_LIBRARIES})

find_package(std_msgs REQUIRED)
include_directories(${std_msgs_INCLUDE_DIRS})
list(APPEND Libs ${std_msgs_LIBRARIES})

# custom target that builds amrl_msgs
add_custom_target(amrl_msgs
  COMMAND ${CMAKE_COMMAND} -E env "ROS_PACKAGE_PATH=${CMAKE_CURRENT_SOURCE_DIR}/amrl_msgs:$ENV{ROS_PACKAGE_PATH}" make
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/amrl_msgs
)
include_directories(amrl_msgs/msg_gen/cpp/include)
add_dependencies(client amrl_msgs)
add_dependencies(test_client amrl_msgs)

add_subdirectory(googletest)

target_link_libraries(client ${Libs})
target_link_libraries(test_client ${Libs} gtest gtest_main)
