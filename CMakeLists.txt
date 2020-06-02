# https://github.com/gerkey/ros1_external_use/blob/master/ros1_msgs/CMakeLists.txt
cmake_minimum_required(VERSION 3.1.0)
project(client)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)
add_executable(client)
add_subdirectory(src)
include_directories(client include)

find_package(Boost REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})
target_link_libraries(client ${Boost_LIBRARIES})

# Build an executable that uses an existing ROS msg: find_package() the ROS
# package that contains the message, then use its reported include directories.
find_package(roscpp REQUIRED)
include_directories(${roscpp_INCLUDE_DIRS})
target_link_libraries(client ${roscpp_LIBRARIES})

find_package(std_msgs REQUIRED)
include_directories(${std_msgs_INCLUDE_DIRS})
target_link_libraries(client ${std_msgs_LIBRARIES})
