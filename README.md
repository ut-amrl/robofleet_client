# robofleet_client

*The RoboFleet 2.0 Robot Client*

## Dependencies

* Make sure to clone submodules, or `git submodule init` and `git submodule update`
* C++ compiler for std >= 11
* CMake
* ROS Melodic
* Qt5

## Configuration

The client must be configured before building.
1. `cp src/config.example.hpp src/config.hpp`
2. Edit parameters in `src/config.hpp`

## Building

* `make` to build
* `make run` to run
* `make format` to run clang-format

### Using namespaces

1. Read the ROS wiki on [names](http://wiki.ros.org/Names) and [name remapping](http://wiki.ros.org/Remapping%20Arguments).
2. Launch with `ROS_NAMESPACE` set, e.g. `ROS_NAMESPACE="/ut/testbot" make run`

Note that if you run two client nodes on one machine (even in different namespaces), it is possible to create a feedback loop by client B receiving client A's messages from the server, and then re-publishing them to client A's topic.

## Development

* Code completion for amrl_msgs
  * Build once to generate amrl_msgs
  * Add `amrl_msgs/msg_gen/cpp/include` to your include paths
