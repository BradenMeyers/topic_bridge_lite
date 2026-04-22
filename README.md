# Network Bridge
[![CI](https://github.com/brow1633/network_bridge/actions/workflows/CI.yml/badge.svg)](https://github.com/brow1633/network_bridge/actions/workflows/CI.yml)

**Network Bridge** is a lightweight ROS2 node designed for robust communication between robotic systems over arbitrary network protocols. Supporting UDP and TCP protocols out of the box, this packages seamlessly bridges ROS2 topics across networks, facilitating effective remote communications between a base station and robotic systems, or between multiple robotic systems.

## Installation
<!-- ### Installation via apt
Install with:
```
sudo apt install ros-<distro>-network-bridge
``` -->

### Building from Source
Simply clone the repository into your ROS2 workspace and build with `colcon build`.

## Usage

### Demo
#### UDP
```
# In first terminal
ros2 launch network_bridge socket_launch.py

# In second terminal (Could be on another device)
ros2 launch network_bridge socket_launch.py namespace:=socket2

# In third terminal for publishing topic
ros2 run network_bridge dummy_publisher.py

ros2 topic echo /vector/out1
```

#### TCP
```
# In first terminal
ros2 launch network_bridge socket_launch.py params_file:=/path/to/pkg/config/tcp.yaml

# In second terminal (Could be on another device)
ros2 launch network_bridge socket_launch.py namespace:=socket2 params_file:=/path/to/pkg/config/tcp.yaml

# In third terminal for publishing topic
ros2 run network_bridge dummy_publisher.py

ros2 topic echo /vector/out1
```

#### Bluetooth
```
# On the server device (waits for incoming connection)
ros2 launch network_bridge bluetooth_launch.py

# On the client device — set bt_role: "client" and bt_mac_address in config/bluetooth.yaml
ros2 launch network_bridge bluetooth_launch.py namespace:=bt2

# Publish a topic and verify it bridges across
ros2 run network_bridge dummy_publisher.py

ros2 topic echo /vector/out1
```

#### Radio (XBee)
```
# On the robot
ros2 launch network_bridge radio_launch.py

# On the base station (update xbee_port and xbee_dest_address in config/radio.yaml)
ros2 launch network_bridge radio_launch.py namespace:=radio2

# Publish a topic and verify it bridges across
ros2 run network_bridge dummy_publisher.py

ros2 topic echo /vector/out1
```

### Configuration
Simply setup the network interface parameters and list your desired topics to get started.  If you are using UDP over cellular data, it is recommended to setup a VPN to facilitate connection.  Also, please note that **no encryption** occurs within this package.  Currently, if you would like encryption, you must use a VPN.

See `config/Udp1.yaml` for a description of all parameters, as well as the TCP example configuration files.
#### Minimal Example
The following configuration examples demonstrate a robot sending a message on `/gps/fix` over UDP to a basestation that will then re-publish the message.  This works seamlessly on all message types, so long as they are built and sourced on both ends of the transmission.
#### Robot
```
/udp_sender:
  ros__parameters:
    UdpInterface:
      local_address: "192.168.1.2"
      receive_port: 5001
      remote_address: "192.168.1.3"
      send_port: 5000

    topics:
      - "/gps/fix"
```
#### Base Station
```
/udp_receiver:
  ros__parameters:
    UdpInterface:
      local_address: "192.168.1.3"
      receive_port: 5000
      remote_address: "192.168.1.2"
      send_port: 5001
```
#### Addressing
When `use_addressing: true` is set, each node is assigned a `node_addr` in its topics yaml (e.g. `node_addr: 1`). Every outbound frame is stamped with the sender's address, and the receiver filters frames by `dst_addr`.

For RX topic entries, `src_addr` controls which sender's frames are accepted:
- **Omitted or `src_addr: 0`** — accepts frames from *any* source (wildcard). This is the default and works for most setups.
- **`src_addr: 0x01`** — only accepts frames originating from the node with `node_addr: 1`.

> **Note:** A common mistake is setting `src_addr` on an RX entry to a non-zero value when the sending node has a different `node_addr`, which silently drops all frames. When in doubt, leave `src_addr` unset.

#### Special case: TF
The TF topic `/tf` or `/tf_static` are handled as a special cases. The subscriber side will listen to all TF messages, accumulate them
(similarly to a TF buffer) and send all of them at the specified rate. The behavior can be disabled or forced using the `is_tf` configuration.

If some TFs need to be excluded or if the list of TFs to include is finite, one can use the include and exclude regex parameters. A transform is matched (hence excluded or included) if either the `frame_id` or `child_frame_id` are matching a pattern.
```
/udp_sender:
  ros__parameters:
    UdpInterface:
      local_address: "192.168.1.2"
      receive_port: 5001
      remote_address: "192.168.1.3"
      send_port: 5000

    topics:
      - "/prefix/tf"
      - "/tf_static"

    /prefix/tf:
      - is_tf: True
      - is_static_tf: False
      - rate: 10.

    /tf_static:
      - rate: 1.0
      - is_static_tf: True
      - exclude: ["standoff.*", "spacer.*", ".*wheel_link", ".*cliff.*"]
```

### Choice of protocol
- **UDP**: Use UDP for low-latency, high-throughput communications, where occasional data loss is tolerable. Ideal for real-time telemetry data like sensor streams.
- **TCP**: Opt for TCP when data integrity and reliability are critical. This ensures that control commands and state transitions are reliably delivered, though with potentially higher latency.

Network protocols are implemented as pluginlib plugins, allowing the creation of arbitrary interfaces using the abstract class `include/network_interfaces/network_interface_base.hpp`.  Any interface that can send and receive bytes could theoretically be implemented, including protocols that go beyond point-to-point communication, such as ZMQ.  Please consider opening a pull request if you implement a new network interface.

### Tuning
This node can be launched with logger level DEBUG, which provides useful information for tuning the compression, rate and stale message parameters.  For each message that is sent, the receiving side will output the number of bytes received, the decompressed size in bytes and the transmission delay.

### Contributing
Thank you for considering contributing!

#### Code Formatting
Python code is formatted with `black`, and C++ is formatted with `uncrustify`.

#### Pre-commit hooks
To ease the friction of linting, there are pre-commit hooks that you can install:

```bash
sudo apt install pre-commit

pre-commit run -a # Run on all files manually

pre-commit install # Run on commit automatically
```

which will reformat code automatically when you commit changes.

## Acknowledgements
This package was developed for use in the Indy Autonomous Challenge by the Purdue AI Racing team.  Inspiration was taken from mqtt_client (https://github.com/ika-rwth-aachen/mqtt_client/).
