#!/usr/bin/env python3
"""explore_map_saver.py - save rtabmap's 2D map when frontier exploration ends.

Started by launch.py when explore:=true.

How "exploration done" is detected: explore_lite (m-explore-ros2, main
branch) publishes explore_lite_msgs/msg/ExploreStatus on /explore/status
(transient local QoS). When no frontiers are left ("No frontiers found,
stopping." or "All frontiers traversed/tried out, stopping." in its log) it
publishes status "exploration_complete". This node waits for that message
and then runs:

    ros2 run nav2_map_server map_saver_cli -t /map \\
        -f <maps_dir>/<map_name>_<YYYY-mm-dd_HH-MM-SS> \\
        --ros-args -p save_map_timeout:=<save_timeout>

which writes <name>.pgm (the picture) and <name>.yaml (resolution, origin,
thresholds) into maps_dir.

Every "exploration_complete" saves a new, time-stamped map, so if you resume
exploration (ros2 topic pub --once /explore/resume std_msgs/msg/Bool
"{data: true}") and it finishes again, you get a second file.

Parameters:
    maps_dir      folder for the map files (created if missing)
    map_name      file name prefix (default "apartment")
    map_topic     occupancy grid to save (default "/map", rtabmap's grid)
    save_timeout  seconds map_saver_cli waits for the map (default 10.0)
"""

import os
import subprocess
import threading
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from explore_lite_msgs.msg import ExploreStatus


class ExploreMapSaver(Node):

    def __init__(self):
        super().__init__('explore_map_saver')
        self.maps_dir = self.declare_parameter(
            'maps_dir', '/workspaces/ros2-dev/src/stereo_eye_launcher/maps').value
        self.map_name = self.declare_parameter('map_name', 'apartment').value
        self.map_topic = self.declare_parameter('map_topic', '/map').value
        self.save_timeout = float(self.declare_parameter('save_timeout', 10.0).value)

        # Same QoS as explore_lite's publisher (depth 10, transient local), so
        # a status sent just before this node started is still received.
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(ExploreStatus, '/explore/status', self.on_status, qos)
        self.saving = False  # True while map_saver_cli runs
        self.get_logger().info(
            f'Waiting for "exploration_complete" on /explore/status; '
            f'maps go to {self.maps_dir}')

    def on_status(self, msg):
        self.get_logger().info(f'Exploration status: {msg.status}')
        if msg.status != ExploreStatus.EXPLORATION_COMPLETE:
            return
        if self.saving:
            self.get_logger().info('A map save is already running, skipping.')
            return
        self.saving = True
        # Run the saver in a thread so this node keeps spinning meanwhile.
        threading.Thread(target=self.save_map, daemon=True).start()

    def save_map(self):
        try:
            os.makedirs(self.maps_dir, exist_ok=True)
            stamp = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
            path = os.path.join(self.maps_dir, f'{self.map_name}_{stamp}')
            cmd = [
                'ros2', 'run', 'nav2_map_server', 'map_saver_cli',
                '-t', self.map_topic,
                '-f', path,
                '--ros-args', '-p', f'save_map_timeout:={self.save_timeout}',
            ]
            self.get_logger().info('Exploration complete, saving map: ' + ' '.join(cmd))
            result = subprocess.run(cmd, timeout=self.save_timeout + 30.0)
            if result.returncode == 0:
                self.get_logger().info(f'Map saved: {path}.pgm and {path}.yaml')
            else:
                self.get_logger().error(
                    f'map_saver_cli failed (exit code {result.returncode}). '
                    f'Save by hand: ros2 run nav2_map_server map_saver_cli -f {path}')
        except Exception as error:  # report anything, never crash the launch
            self.get_logger().error(f'Map save failed: {error}')
        finally:
            self.saving = False


def main():
    rclpy.init()
    node = ExploreMapSaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
