# robofleet_client

*The Robofleet 2.0 Robot Client*

## Dependencies

* Make sure to clone submodules, or `git submodule init` and `git submodule update`
* C++ compiler for std >= 11
* CMake
* ROS Melodic
* Qt5 >= 5.5
* libqt5websockets5-dev

## Configuration

The client must be configured before building.
1. `cp src/config.example.hpp src/config.hpp`
2. Edit parameters in `src/config.hpp`

Below is more information about configuration options in `src/config.hpp`.

### Topic configuration
You must tell the client exactly which ROS topics to exchange with the Robofleet server. To set which topics the client sends and receives, you can follow the examples in the `configure_msg_types()` function.

Keep in mind that all ROS topic [names][names] are [resolved][name resolution], meaning most importantly that topics not beginning with a `/` are prefixed with the ROS namespace.

There are two available configuration builders:

`SendLocalTopic<T>` - subscribe to messages on a local ROS topic and send them to the server. `T` is a ROS message class.
* `.from` - the name of the local ROS topic
* `.to` - the name of the remote ROS topic (can be the same; allows for remapping)
* `.priority` - larger priority means the topic will receive more bandwidth. Priority is an arbitrary positive value where the ratio of priorities between topics determines how much bandwidth each is allocated.
* `.no_drop` - `true` means to never drop messages on this topic and queue them instead. **Can cause lag** and should only be used for messages that are sent infrequently. Messages on topics flagged as `no_drop` are scheduled in FIFO order and always take priority over other messages.
* `.rate_limit_hz` - set the maximum rate of message sending to a given value in hz (default unlimited). This is only required if you want to impose a hard limit on message frequency; you can always use `priority` to adjust how much bandwidth is allocated to each topic.

`ReceiveRemoteTopic<T>` - receive messages from the server and publish them to a local ROS topic. `T` is a ROS message class.
* `.from` - the name of the remote ROS topic
* `.to` - the name of the local ROS topic

### Direct mode

By default, the Robofleet robot client runs a WebSocket client, which connects to an instance of `robofleet_server`. You can then use `robofleet_webviz` to connect to the server and interact with connected robots. In some cases, you may not need the features provided by `robofleet_server`, or you may not want to run the Node.js application. In this case, you can enable "direct mode" in the robot client, which causes it to accept connections as a WebSocket server instead of acting as a WebSocket client. You can then use `robofleet_webviz` to connect directly to the robot running the robot client, instead of connecting to an intermediary instance of `robofleet_server`.

Direct mode does not have features such as authentication or subscriptions, nor does it currently support secure WebSockets. It simply broadcasts each incoming message to all other clients, as well as sending messages from local ROS topics as usual.

To switch to direct mode, set the `direct_mode` flag in `src/config.hpp` and make sure to choose a `direct_mode_port`. You may need to tune `direct_mode_bytes_per_sec`, increasing it to prevent dropped messages or decreasing it to prevent time lag when large amounts of message data are sent.

## Building

* `make` to build
* `ROS_NAMESPACE="robot_name" make run` to run
* `make format` to run clang-format

### Using namespaces

1. Read the ROS wiki on [names][names] and [name resolution][name resolution].
2. Launch with `ROS_NAMESPACE` set, e.g. `ROS_NAMESPACE="ut/testbot" make run` or by using `export` to set the environment variable beforehand.

Note that if you run two client nodes on one machine (even in different namespaces), it is possible to create a feedback loop by client B receiving client A's messages from the server, and then re-publishing them to client A's topic.

## Development

* Code completion for amrl_msgs
  * Build once to generate amrl_msgs
  * Add `amrl_msgs/msg_gen/cpp/include` to your include paths
  * VS Code editor settings are included
* Debugging
  1. `cd build && cmake -DCMAKE_BUILD_TYPE=Debug && make -j`
  1. Either launch `(gdb) Launch` or `(gdb) Tests` in VSCode or use `gdb build/client`

[names]: https://wiki.ros.org/Names#Resolving
[name resolution]: https://wiki.ros.org/Names#Resolving
