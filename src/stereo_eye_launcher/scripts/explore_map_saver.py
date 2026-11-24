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

Too-early "complete" (start-up guard):
explore_lite starts searching as soon as Nav2 is up, while rtabmap's map is
still just the cameras' first view (the robot has not moved yet). If it finds
no usable frontier in that first view it reports "exploration_complete" within
a second, and the robot never moves (Kunal's run: started 1964.37, complete
1964.45, map 177 x 164 cells built from ONE rtabmap node). explore_lite
(m-explore-ros2 326cf8a) has no "start paused" or "wait for map" option, but
it does restart when it gets True on /explore/resume (std_msgs/Bool), even
after it has stopped by itself.
So a "complete" is treated as TOO EARLY if it comes less than
min_explore_time seconds after "exploration_started", or while the map knows
less than min_known_area_m2 square metres (free + wall cells). Then this node:
  1. prints "[EXPLORE] Map still tiny (... m2 known after ... s), looking
     around and restarting exploration (try n of max_restarts)",
  2. turns the robot once on the spot (look_around_angle, default a full
     turn, at look_around_speed 0.3 rad/s, about 21 s) by publishing /cmd_vel,
     so the cameras see the whole room and the map grows. Nav2 has no goal
     at this point, so nothing else is driving the robot, and
     stuck_recovery.py stays idle because exploration is not running,
  3. sends True on /explore/resume, so explore_lite searches again.
After max_restarts such tries it saves the map anyway (with a warning).

Parameters:
    maps_dir           folder for the map files (created if missing)
    map_name           file name prefix (default "apartment")
    map_topic          occupancy grid to save (default "/map", rtabmap's grid)
    save_timeout       seconds map_saver_cli waits for the map (default 10.0)
    min_explore_time   s; a "complete" sooner than this is too early (60.0)
    min_known_area_m2  m2; a "complete" with less known map is too early (4.0)
    max_restarts       how many times to look around and restart (3)
    look_around_angle  rad turned on the spot before a restart (6.28 = 360 deg;
                       0 = no turn, just wait restart_wait seconds)
    look_around_speed  rad/s of that turn (0.3; visual odometry copes with 0.4)
    restart_wait       s to wait after the turn before restarting (2.0), so
                       rtabmap has added the new views to /map
    cmd_vel_topic      where the turn is sent (default "/cmd_vel")
"""

import os
import subprocess
import threading
from datetime import datetime

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from explore_lite_msgs.msg import ExploreStatus
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool


class ExploreMapSaver(Node):

    def __init__(self):
        super().__init__('explore_map_saver')
        self.maps_dir = self.declare_parameter(
            'maps_dir', '/workspaces/ros2-dev/src/stereo_eye_launcher/maps').value
        self.map_name = self.declare_parameter('map_name', 'apartment').value
        self.map_topic = self.declare_parameter('map_topic', '/map').value
        self.save_timeout = float(self.declare_parameter('save_timeout', 10.0).value)
        # Start-up guard (see the header)
        p = self.declare_parameter
        self.min_explore_time = float(p('min_explore_time', 60.0).value)     # s
        self.min_known_area_m2 = float(p('min_known_area_m2', 4.0).value)    # m2
        self.max_restarts = int(p('max_restarts', 3).value)
        self.look_around_angle = float(p('look_around_angle', 6.28).value)   # rad
        self.look_around_speed = float(p('look_around_speed', 0.3).value)    # rad/s
        self.restart_wait = float(p('restart_wait', 2.0).value)              # s
        self.cmd_vel_topic = p('cmd_vel_topic', '/cmd_vel').value

        self.started_at = None     # time of the first "exploration_started"
        self.restarts = 0          # look-around restarts done so far
        self.restarting = False    # True while turning / waiting to restart
        self.turn_end = None
        self.resume_at = None
        self.known_area_m2 = 0.0   # from the latest map
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.resume_pub = self.create_publisher(Bool, '/explore/resume', 10)
        map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)  # rtabmap latches /map
        self.create_subscription(OccupancyGrid, self.map_topic, self.on_map, map_qos)
        self.create_timer(0.1, self.tick)  # 10 Hz: drives the look-around turn

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

    def on_map(self, msg):
        # Known = free (0) or occupied (1-100); unknown is -1. array.count is
        # fast even for big maps (rtabmap republishes /map about once a second).
        known = len(msg.data) - msg.data.count(-1)
        self.known_area_m2 = known * msg.info.resolution ** 2

    def now(self):
        return self.get_clock().now()

    def too_early(self):
        """Reason string if this "complete" looks like a start-up false alarm, else ''."""
        if self.started_at is None:
            return ''  # never saw "started": nothing to compare with, trust it
        age = (self.now() - self.started_at).nanoseconds / 1e9
        if age < self.min_explore_time or self.known_area_m2 < self.min_known_area_m2:
            return f'{self.known_area_m2:.1f} m2 known after {age:.0f} s'
        return ''

    def on_status(self, msg):
        self.get_logger().info(f'Exploration status: {msg.status}')
        if msg.status == ExploreStatus.EXPLORATION_STARTED and self.started_at is None:
            self.started_at = self.now()
        if msg.status != ExploreStatus.EXPLORATION_COMPLETE:
            return
        if self.restarting:
            return  # already looking around; the resume comes after the turn
        reason = self.too_early()
        if reason and self.restarts < self.max_restarts:
            self.restarts += 1
            self.get_logger().warn(
                f'[EXPLORE] Map still tiny ({reason}), looking around and restarting '
                f'exploration (try {self.restarts} of {self.max_restarts})')
            self.restarting = True
            turn_s = abs(self.look_around_angle) / max(abs(self.look_around_speed), 1e-3)
            self.turn_end = self.now() + Duration(seconds=turn_s)
            self.resume_at = self.turn_end + Duration(seconds=self.restart_wait)
            return
        if reason:
            self.get_logger().warn(
                f'[EXPLORE] Map is still small ({reason}) after {self.max_restarts} '
                'restarts; saving it anyway.')
        if self.saving:
            self.get_logger().info('A map save is already running, skipping.')
            return
        self.saving = True
        # Run the saver in a thread so this node keeps spinning meanwhile.
        threading.Thread(target=self.save_map, daemon=True).start()

    def tick(self):
        if not self.restarting:
            return
        now = self.now()
        if now < self.turn_end:
            cmd = Twist()
            cmd.angular.z = abs(self.look_around_speed)
            self.cmd_pub.publish(cmd)  # turn on the spot (Nav2 is idle now)
        elif now < self.resume_at:
            if self.look_around_angle != 0.0:
                self.cmd_pub.publish(Twist())  # stop and let rtabmap catch up
        else:
            self.restarting = False
            self.get_logger().info(
                f'[EXPLORE] Restarting exploration ({self.known_area_m2:.1f} m2 known now)')
            self.resume_pub.publish(Bool(data=True))

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
