#!/usr/bin/env python3
"""stuck_recovery.py - notice when the robot is stuck and get it free again.

Started by launch.py only when explore:=true.

When is the robot "stuck"? Either of:
  1. Too close (the MAIN trigger): the front time-of-flight (ToF) range
     sensor (tof_topic, default /front_tof, sensor_msgs/msg/LaserScan from a
     Gazebo gpu_lidar, 0.02-0.30 m range) measures something closer than
     tof_too_close_m (default 0.04 m, was 0.15 m), i.e. the robot's nose is
     practically touching it. A reading counts if it is finite,
     >= range_min and < tof_too_close_m; tof_too_close_m <= 0 means "use the
     scan's range_max" instead. "No echo" readings (inf) never count. The
     recovery starts at once, no waiting. Between 0.04 and 0.30 m the ToF
     sensor only feeds Nav2's local costmap (nav2_params.yaml), which steers
     the robot away; this node fires only when that was not enough.
     Note: this needs the scan's range_min (0.02 m in MechaX's xacro) to be
     below tof_too_close_m, otherwise no reading can ever count.
  2. Pushing without moving (a CONSERVATIVE backup): the speed command that
     reaches the robot (/cmd_vel, after Nav2's velocity smoother) has asked
     to drive FORWARD faster than min_cmd_speed (0.08 m/s, was 0.05) without
     a single slower command for stuck_time (5 s, was 2 s), but the VISUAL
     odometry pose (/odom from stereo_odometry) moved less than min_progress
     (5 cm, was 3 cm; the command alone would have moved it >= 0.4 m).
     Turning in place (linear speed ~0), slowing down, hesitating or Nav2's
     own back-up manoeuvre (negative speed) all reset the timer, so they can
     never make the robot reverse.
     Wheel odometry (/wheel_odom) is not used on purpose: when the wheels slip
     against a wall, wheel odometry still "drives", the cameras do not.
     If visual odometry sends nothing at all while pushing, that only counts
     as "not moving" with treat_lost_odom_as_stuck:=true (default now False:
     losing visual tracking is not proof of being stuck, and the wall-in-
     front case that loses tracking is caught by the ToF trigger).

What happens then:
  The same reading cannot fire twice: ToF readings are ignored while a
  recovery runs and during the cooldown after it.

  a. Send one zero speed command at once (and keep sending zeros until the
     back-up starts), so the robot stops right away instead of driving on
     while Nav2 processes the abort. Nav2's own last commands are zero or
     decelerating at that point, so the two do not fight.
     Make explore_lite give up on its current frontier. Simply cancelling the
     Nav2 goal does NOT work: explore_lite (m-explore-ros2 326cf8a,
     Explore::reachedGoal) ignores CANCELED goals ("might be because
     exploration stopped") and later sends the same frontier again. It only
     blacklists ABORTED goals. So this node sends its OWN navigate_to_pose
     goal (the robot's current pose). Nav2's bt_navigator accepts it as a
     preemption, and Nav2's action server then ends explore_lite's goal as
     ABORTED (nav2_util SimpleActionServer::accept_pending_goal), so
     explore_lite blacklists that frontier.
  b. After preempt_wait (1 s, so the abort is processed first), pause
     explore_lite (False on /explore/resume). That also cancels our own goal,
     so Nav2 stops sending speed commands. Then wait settle_time (1.2 s):
     Nav2's velocity smoother keeps sending zero commands for its
     velocity_timeout (1 s) after Nav2 stops, which would fight our commands.
  c. Back up backup_distance (0.2 m) slowly, then turn rotate_angle (~90 deg),
     alternating left and right, by publishing /cmd_vel directly.
  d. Stop, resume explore_lite (True on /explore/resume) if it was exploring,
     and ignore new "stuck" signals for cooldown seconds while Nav2 starts
     driving again.

Backing up is open-loop (speed x time) and the cameras cannot see behind the
robot, so keep backup_distance small.

All thresholds are ROS parameters (see declare_parameter calls below).
"""

import math
from collections import deque

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time

import tf2_ros
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

from explore_lite_msgs.msg import ExploreStatus

# Phases of the recovery
IDLE = 'idle'
PREEMPT = 'preempt'      # waiting for our goal to replace explore_lite's goal
SETTLE = 'settle'        # explore paused, waiting for Nav2's last commands to stop
BACKUP = 'backup'
ROTATE = 'rotate'
COOLDOWN = 'cooldown'


class StuckRecovery(Node):

    def __init__(self):
        super().__init__('stuck_recovery')
        p = self.declare_parameter
        # Topics and frames
        self.cmd_vel_topic = p('cmd_vel_topic', '/cmd_vel').value
        self.odom_topic = p('odom_topic', '/odom').value
        self.tof_topic = p('tof_topic', '/front_tof').value
        self.tof_too_close_m = p('tof_too_close_m', 0.04).value     # m (was 0.15); <= 0: use range_max
        self.map_frame = p('map_frame', 'map').value
        self.base_frame = p('base_frame', 'base_footprint').value
        self.nav_action = p('navigate_action', 'navigate_to_pose').value
        # Detection thresholds
        # Conservative on purpose: the ToF trigger is the main one.
        self.min_cmd_speed = p('min_cmd_speed', 0.08).value        # m/s forward (was 0.05)
        self.stuck_time = p('stuck_time', 5.0).value                # s (was 2.0)
        self.min_progress = p('min_progress', 0.05).value           # m (was 0.03)
        self.odom_timeout = p('odom_timeout', 1.0).value            # s
        # was True; see the header: lost tracking alone is not "stuck"
        self.treat_lost_odom_as_stuck = p('treat_lost_odom_as_stuck', False).value
        # Recovery motion
        self.backup_distance = p('backup_distance', 0.2).value      # m
        self.backup_speed = p('backup_speed', 0.05).value           # m/s
        self.rotate_angle = p('rotate_angle', 1.57).value           # rad
        self.rotate_speed = p('rotate_speed', 0.3).value            # rad/s
        self.preempt_wait = p('preempt_wait', 1.0).value            # s
        self.settle_time = p('settle_time', 1.2).value              # s
        self.cooldown = p('cooldown', 10.0).value                   # s

        # State
        self.phase = IDLE
        self.phase_end = None          # Time when the current phase ends
        self.turn_sign = 1.0           # alternates left / right
        self.cmd_since = None          # Time since when we are commanded to move
        self.last_cmd_time = None
        self.odom_samples = deque()    # (Time received, x, y) of visual odometry
        self.too_close = None          # (Time, distance m) of the latest too-close ToF reading
        self.explore_status = None
        self.paused_explore = False

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.resume_pub = self.create_publisher(Bool, '/explore/resume', 10)
        self.create_subscription(Twist, self.cmd_vel_topic, self.on_cmd_vel, 10)
        self.create_subscription(Odometry, self.odom_topic, self.on_odom, 10)
        status_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(ExploreStatus, '/explore/status', self.on_status, status_qos)
        self.create_subscription(LaserScan, self.tof_topic, self.on_tof, 10)

        self.nav_client = ActionClient(self, NavigateToPose, self.nav_action)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_timer(0.1, self.tick)  # 10 Hz: detection and recovery motion
        self.get_logger().info(
            f'Watching for a stuck robot: {self.tof_topic} closer than '
            f'{self.tof_too_close_m} m, or {self.cmd_vel_topic} forward > '
            f'{self.min_cmd_speed} m/s for {self.stuck_time} s '
            f'with {self.odom_topic} moving < {self.min_progress} m')

    # ---------- inputs ----------

    def on_tof(self, msg):
        if self.phase != IDLE:
            return  # recovering or cooling down: do not re-fire on the same obstacle
        limit = self.tof_too_close_m if self.tof_too_close_m > 0.0 else msg.range_max
        valid = [r for r in msg.ranges
                 if math.isfinite(r) and r >= msg.range_min and r < limit]
        if valid:
            self.too_close = (self.now(), min(valid))

    def on_cmd_vel(self, msg):
        if self.phase != IDLE:
            return  # our own recovery commands, or Nav2 winding down
        now = self.now()
        self.last_cmd_time = now
        # Only a steady FORWARD push counts. Turning in place (linear ~0),
        # slowing down or backing up (negative) restarts the timer.
        if msg.linear.x > self.min_cmd_speed:
            if self.cmd_since is None:
                self.cmd_since = now
        else:
            self.cmd_since = None

    def on_odom(self, msg):
        now = self.now()
        self.odom_samples.append((now, msg.pose.pose.position.x, msg.pose.pose.position.y))
        keep = Duration(seconds=self.stuck_time + 2.0)
        while self.odom_samples and now - self.odom_samples[0][0] > keep:
            self.odom_samples.popleft()

    def on_status(self, msg):
        self.explore_status = msg.status

    # ---------- helpers ----------

    def now(self):
        return self.get_clock().now()

    def exploring(self):
        return self.explore_status in (ExploreStatus.EXPLORATION_STARTED,
                                       ExploreStatus.EXPLORATION_IN_PROGRESS)

    def robot_pose(self):
        """Robot pose in the map frame as PoseStamped, or None."""
        try:
            t = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, Time())
        except tf2_ros.TransformException:
            return None
        pose = PoseStamped()
        pose.header.frame_id = self.map_frame
        pose.header.stamp = self.now().to_msg()
        pose.pose.position.x = t.transform.translation.x
        pose.pose.position.y = t.transform.translation.y
        pose.pose.orientation = t.transform.rotation
        return pose

    def publish_cmd(self, linear=0.0, angular=0.0):
        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        self.cmd_pub.publish(cmd)

    def pushing_without_moving(self, now):
        """True if commanded to drive for stuck_time while visual odometry stays put."""
        if self.cmd_since is None or self.last_cmd_time is None:
            return False, ''
        if now - self.last_cmd_time > Duration(seconds=0.5):
            self.cmd_since = None  # commands stopped
            return False, ''
        if now - self.cmd_since < Duration(seconds=self.stuck_time):
            return False, ''
        if not self.odom_samples or now - self.odom_samples[-1][0] > Duration(seconds=self.odom_timeout):
            if self.treat_lost_odom_as_stuck:
                return True, 'driving but visual odometry is lost'
            return False, ''
        window_start = now - Duration(seconds=self.stuck_time)
        older = [s for s in self.odom_samples if s[0] <= window_start]
        if not older:
            return False, ''  # not enough history yet
        _, x0, y0 = older[-1]
        _, x1, y1 = self.odom_samples[-1]
        moved = math.hypot(x1 - x0, y1 - y0)
        if moved < self.min_progress:
            return True, f'pushing into a wall (moved only {moved * 100:.1f} cm in {self.stuck_time:.0f} s)'
        return False, ''

    # ---------- main loop ----------

    def tick(self):
        now = self.now()
        if self.phase == IDLE:
            too_close = (self.too_close is not None and
                         now - self.too_close[0] < Duration(seconds=0.5))
            stuck, reason = self.pushing_without_moving(now)
            if too_close:
                cm = self.too_close[1] * 100.0
                self.start_recovery('[TOO CLOSE]', f'Something is {cm:.0f} cm in front of the robot')
            elif stuck:
                self.start_recovery('[STUCK]', f'Robot {reason}')
        elif self.phase == PREEMPT:
            self.publish_cmd()  # keep the robot stopped while Nav2 lets go
            if now >= self.phase_end:
                if self.exploring():
                    self.resume_pub.publish(Bool(data=False))  # pause; cancels our goal too
                    self.paused_explore = True
                self.enter(SETTLE, self.settle_time)
        elif self.phase == SETTLE:
            self.publish_cmd()  # same: zeros only, until the back-up starts
            if now >= self.phase_end:
                self.enter(BACKUP, self.backup_distance / max(self.backup_speed, 1e-3))
        elif self.phase == BACKUP:
            if now >= self.phase_end:
                self.enter(ROTATE, self.rotate_angle / max(self.rotate_speed, 1e-3))
            else:
                self.publish_cmd(linear=-abs(self.backup_speed))
        elif self.phase == ROTATE:
            if now >= self.phase_end:
                self.publish_cmd()  # stop
                if self.paused_explore:
                    self.resume_pub.publish(Bool(data=True))
                    self.paused_explore = False
                    self.get_logger().info('[STUCK] Recovery done, exploration continues.')
                else:
                    self.get_logger().info('[STUCK] Recovery done.')
                self.turn_sign = -self.turn_sign
                self.enter(COOLDOWN, self.cooldown)
            else:
                self.publish_cmd(angular=self.turn_sign * abs(self.rotate_speed))
        elif self.phase == COOLDOWN:
            if now >= self.phase_end:
                self.cmd_since = None
                self.too_close = None
                self.phase = IDLE

    def enter(self, phase, seconds):
        self.phase = phase
        self.phase_end = self.now() + Duration(seconds=seconds)

    def start_recovery(self, tag, what):
        pose = self.robot_pose()
        where = (f'({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})'
                 if pose is not None else '(position unknown)')
        self.get_logger().warn(
            f'{tag} {what} at {where}: backing up and choosing another frontier')
        self.cmd_since = None
        self.too_close = None
        self.publish_cmd()  # (a) stop now
        # (a) Replace explore_lite's goal with ours so Nav2 aborts it -> blacklisted.
        if pose is not None and self.nav_client.server_is_ready():
            goal = NavigateToPose.Goal()
            goal.pose = pose
            self.nav_client.send_goal_async(goal)
        else:
            self.get_logger().warn(
                '[STUCK] Could not replace the Nav2 goal (no map pose or Nav2 not ready); '
                'explore_lite may try the same frontier again.')
        self.enter(PREEMPT, self.preempt_wait)


def main():
    rclpy.init()
    node = StuckRecovery()
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
