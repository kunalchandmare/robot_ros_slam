# Learning Notes – Simple ROS 2 Nodes Demo

This document explains the key ROS 2 concepts used in the example.

## 1. Nodes
A **node** is a process that performs computation. In ROS 2 every program you run with `ros2 run` is a node.

- Nodes communicate over **topics**, **services**, or **actions**.
- In this demo we only use **topics** (the simplest and most common mechanism).

## 2. Topics, Publishers and Subscribers
- A **topic** is a named channel (e.g. `/battery_level`).
- A **publisher** sends messages to a topic.
- A **subscriber** receives messages from a topic.
- Multiple nodes can publish to or subscribe from the same topic.

In this example:
- `battery_publisher` (Python) publishes an `Int32` on `/battery_level`.
- `battery_processor` (Python) and `battery_listener` (C++) both subscribe to that topic.

## 3. Messages
ROS 2 uses strongly typed messages. We use two standard messages from the `std_msgs` package:
- `std_msgs/msg/Int32` — a simple integer
- `std_msgs/msg/String` — a simple string

## 4. Python nodes (rclpy)
Python nodes use the `rclpy` library.

Typical structure:
```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node_name')
        # create publisher / subscriber / timer here

def main():
    rclpy.init()
    node = MyNode()
    rclpy.spin(node)   # keeps the node alive and processes callbacks
    node.destroy_node()
    rclpy.shutdown()
```

Key points in the code:
- `self.create_publisher(msg_type, topic_name, qos)` — creates a publisher.
- `self.create_subscription(msg_type, topic_name, callback, qos)` — creates a subscriber.
- `self.create_timer(period, callback)` — calls a function periodically.
- QoS (Quality of Service) of `10` is a common default (queue size).

## 5. C++ node (rclcpp)
C++ nodes use the `rclcpp` library. The structure is very similar:

```cpp
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

class MyNode : public rclcpp::Node {
public:
  MyNode() : Node("my_node_name") {
    subscription_ = this->create_subscription<std_msgs::msg::Int32>(
      "battery_level", 10, std::bind(&MyNode::callback, this, std::placeholders::_1));
  }
private:
  void callback(const std_msgs::msg::Int32::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received: %d", msg->data);
  }
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;
};
```

## 6. Package structure
```
simple_nodes_demo/
├── package.xml          # package metadata and dependencies
├── setup.py             # Python package build (ament_python)
├── setup.cfg
├── CMakeLists.txt        # needed for the C++ node
├── simple_nodes_demo/   # Python source
│   ├── battery_publisher.py
│   └── battery_processor.py
└── src/
    └── battery_listener.cpp
```

## 7. Build system
- Python-only packages use `setup.py` + `ament_python`.
- Mixed packages also need a `CMakeLists.txt` so the C++ executable is compiled.
- After `colcon build` the nodes become available via `ros2 run`.

## 8. Next steps to try yourself
1. Change the publish rate or the battery decrease speed.
2. Add a new topic (e.g. temperature).
3. Convert one of the Python nodes to C++ (or vice-versa).
4. Add a simple service (request/response) instead of only topics.
5. Visualize the topics with `rqt_graph` or Foxglove.

Happy learning!
