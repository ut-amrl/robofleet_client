# robofleet_client

*The Robofleet 2.0 Robot Client*

## Dependencies

* Make sure to clone submodules, or `git submodule init` and `git submodule update`
* C++ compiler for std >= 11
* CMake
* ROS Melodic
* Qt5 >= 5.5

## Configuration

The client must be configured before building.
1. `cp src/config.example.hpp src/config.hpp`
2. Edit parameters in `src/config.hpp`

### Direct mode

By default, the Robofleet robot client runs a WebSocket client, which connects to an instance of `robofleet_server`. You can then use `robofleet_webviz` to connect to the server and interact with connected robots. In some cases, you may not need the features provided by `robofleet_server`, or you may not want to run the Node.js application. In this case, you can enable "direct mode" in the robot client, which causes it to accept connections as a WebSocket server instead of acting as a WebSocket client. You can then use `robofleet_webviz` to connect directly to the robot running the robot client, instead of connecting to an intermediary instance of `robofleet_server`.

Direct mode does not have features such as authentication or subscriptions, nor does it currently support secure WebSockets. It simply broadcasts each incoming message to all other clients, as well as sending messages from local ROS topics as usual.

To switch to direct mode, set the `direct_mode` flag in `src/config.hpp` and make sure to choose a `direct_mode_port`.

## Building

* `make` to build
* `ROS_NAMESPACE="robot_name" make run` to run
* `make format` to run clang-format

### Using namespaces

1. Read the ROS wiki on [names](http://wiki.ros.org/Names) and [name remapping](http://wiki.ros.org/Remapping%20Arguments).
2. Launch with `ROS_NAMESPACE` set, e.g. `ROS_NAMESPACE="ut/testbot" make run` or by using `export` to set the environment variable beforehand.

Note that if you run two client nodes on one machine (even in different namespaces), it is possible to create a feedback loop by client B receiving client A's messages from the server, and then re-publishing them to client A's topic.

## Development

* Code completion for amrl_msgs
  * Build once to generate amrl_msgs
  * Add `amrl_msgs/msg_gen/cpp/include` to your include paths
  * VS Code editor settings are included
