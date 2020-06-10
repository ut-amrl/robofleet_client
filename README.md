# robofleet_client

*The RoboFleet 2.0 Robot Client*

## Dependencies

* C++ compiler for std >= 11
* CMake
* ROS Melodic
* Qt5

## Building

* Make sure to clone submodules, or `git submodule init` and `git submodule update`
* `make` to build
* `make run` to run
* `make format` to run clang-format

## Development

* Code completion for amrl_msgs
  * Build once to generate amrl_msgs
  * Add `amrl_msgs/msg_gen/cpp/include` to your include paths
