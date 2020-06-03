cmake_minimum_required(VERSION 3.1.0)
project(client)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Qt
set(CMAKE_AUTOMOC on)
set(CMAKE_AUTORCC on)
set(CMAKE_AUTOUIC on)
set(CMAKE_INCLUDE_CURRENT_DIR on)
find_package(Qt5 COMPONENTS Core WebSockets REQUIRED)

add_executable(client)
add_subdirectory(src)
include_directories(client include)

target_link_libraries(client Qt5::Core Qt5::WebSockets) # also adds includes

# https://github.com/gerkey/ros1_external_use/blob/master/ros1_msgs/CMakeLists.txt
find_package(roscpp REQUIRED)
include_directories(${roscpp_INCLUDE_DIRS})
target_link_libraries(client ${roscpp_LIBRARIES})

find_package(std_msgs REQUIRED)
include_directories(${std_msgs_INCLUDE_DIRS})
target_link_libraries(client ${std_msgs_LIBRARIES})
