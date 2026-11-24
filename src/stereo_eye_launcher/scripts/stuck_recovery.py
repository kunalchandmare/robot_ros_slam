#!/usr/bin/env python3
"""stuck_recovery.py - notice when the robot is stuck and get it free again.

Started by launch.py only when explore:=true. It is the ONLY thing that ever
drives the robot backwards: the explore behaviour tree has no Spin or BackUp
recoveries any more (behavior_trees/explore_navigate_to_pose.xml).

It only acts while explore_lite is exploring (/explore/status is "started"
or "in progress"). When exploration is complete or paused the robot is left
alone. (Kunal's log showed the robot backing up and turning every ~11 s
after exploration had already ended, because of a false ToF reading.)

When is the robot "stuck"? Any of:
  1. [TOO CLOSE] (the main trigger): the front time-of-flight (ToF) range
     sensor (tof_topic, default /front_tof, sensor_msgs/msg/LaserScan from a
     Gazebo gpu_lidar, range_min 0.01 m since MechaX 501ef3f (was 0.02),
     range_max 0.30 m) measures something
     closer than tof_too_close_m (0.04 m) WHILE Nav2 is driving the robot
     forward (a /cmd_vel with linear.x > tof_push_speed, 0.02 m/s, in the last
     0.5 s). Standing still or turning on the spot next to a wall is fine
     (the robot is round), so that never triggers a back-up.
     A reading counts only if it is finite, ABOVE range_min + tof_min_margin
     (now 0.01 + 0.005 m; the scan's own range_min is used) and below
     tof_too_close_m. Why the margin: Gazebo
     (gz-sensors 6, Lidar::Clamp) clamps every finite range below range_min
     UP to exactly range_min. The real "too close" case gives -inf instead
     (gpu_rays shader), so a reading of exactly range_min is never a real
     measurement; it is a rendering artefact (in Kunal's log every reading was
     exactly 0.02 m, the range_min then, even after backing up 20 cm and
     turning 90 deg).
     "No echo" readings (inf) never count either.
  2. [STUCK] pushing without moving (a conservative backup): /cmd_vel (after
     Nav2's velocity smoother) has asked to drive FORWARD faster than
     min_cmd_speed (0.08 m/s) without a single slower command for stuck_time
     (5 s), but the VISUAL odometry pose (/odom from stereo_odometry) moved
     less than min_progress (5 cm). Turning in place, slowing down, pausing or
     reversing all restart the timer.
     Wheel odometry (/wheel_odom) is not used on purpose: when the wheels slip
     against a wall, wheel odometry still "drives", the cameras do not.
     If visual odometry sends nothing at all while pushing, that only counts
     with treat_lost_odom_as_stuck:=true (default False).
  3. [NO PATH] standing too close to a wall to plan: the robot's centre has
     been on a cell of the global costmap that is "inscribed" or "lethal"
     (published value >= 99; the robot circle would touch a wall) for
     inscribed_time (3 s). NavFn treats those cells as walls, so it cannot
     start a path from there and every frontier would fail and be
     blacklisted in turn. Nav2's BackUp recovery used to fix this; now this
     node does it: it moves 0.15 m in whichever direction (forward or back)
     has the lower cost, and does NOT blacklist the frontier.

What happens then (no turning any more; Nav2 turns the robot towards its
next path by itself, so an extra 90 deg turn only added back-and-forth):
  a. Send zero speed at once (and keep sending zeros until the move starts),
     so the robot stops right away.
  b. [TOO CLOSE] and [STUCK] only: make explore_lite give up its current
     frontier. Cancelling the Nav2 goal does NOT work: explore_lite
     (m-explore-ros2 326cf8a, Explore::reachedGoal) ignores CANCELED goals. It
     blacklists ABORTED goals. So this node sends its OWN navigate_to_pose
     goal (the robot's current pose). If explore_lite's goal is running, Nav2
     replaces it (bt_navigator prints "Received goal preemption request") and
     ends it as ABORTED, so explore_lite blacklists that frontier. Our own goal
     is then refused by the tree's "already reached" check ("Goal failed"),
     which is expected.
  c. After preempt_wait (1 s), pause explore_lite (False on /explore/resume),
     wait settle_time (1.2 s) for Nav2's velocity smoother to go quiet, then
     drive backup_distance (0.15 m) backwards slowly (or forwards for
     [NO PATH] when the front is clearer), by publishing /cmd_vel directly.
  d. Stop, resume explore_lite, and ignore new triggers for cooldown seconds.

Repeats: if a trigger fires within repeat_radius (0.5 m) of an earlier one in
the last repeat_window (120 s), it is a repeat. Each repeat backs up 50 %
further (up to 0.3 m) and doubles the cooldown (up to 40 s), so the robot
does not loop at the same spot. ("[TOO CLOSE] (repeat 2 here) ...")

Backing up is open-loop (speed x time) and the cameras cannot see behind the
robot, so keep backup_distance small.

4. [ESCAPE] (only used by frontier_explorer.py, explorer:=own): a request
   on escape_topic (std_msgs/String, the text is the reason), sent when
   several Nav2 goals in a row were aborted while the robot did not move
   (the controller refuses to drive, e.g. "collision ahead" right next to
   furniture). Handled like [NO PATH] without a goal swap (the explorer has
   already blacklisted that area): pause exploration, back up
   backup_distance, then turn escape_turn_angle on the spot towards the side
   with the lower costmap value, and resume. Ignored while a recovery or its
   cooldown is running. explore_lite never publishes it, so with
   explorer:=explore_lite nothing changes.

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
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String

from explore_lite_msgs.msg import ExploreStatus

# Phases of the recovery
IDLE = 'idle'
PREEMPT = 'preempt'      # waiting for our goal to replace explore_lite's goal
SETTLE = 'settle'        # explore paused, waiting for Nav2's last commands to stop
MOVE = 'move'            # backing up (or, for [NO PATH], maybe driving forward)
TURN = 'turn'            # [ESCAPE] only: turning on the spot after backing up
COOLDOWN = 'cooldown'

# nav2_costmap_2d publishes OccupancyGrid values 0-100: 99 = inscribed
# (the robot circle would touch an obstacle), 100 = lethal, -1 = unknown.
INSCRIBED_PUBLISHED = 99


class StuckRecovery(Node):

    def __init__(self):
        super().__init__('stuck_recovery')
        p = self.declare_parameter
        # Topics and frames
        self.cmd_vel_topic = p('cmd_vel_topic', '/cmd_vel').value
        self.odom_topic = p('odom_topic', '/odom').value
        self.tof_topic = p('tof_topic', '/front_tof').value
        self.costmap_topic = p('costmap_topic', '/global_costmap/costmap').value
        self.map_frame = p('map_frame', 'map').value
        self.base_frame = p('base_frame', 'base_footprint').value
        self.nav_action = p('navigate_action', 'navigate_to_pose').value
        # [TOO CLOSE]
        self.tof_too_close_m = p('tof_too_close_m', 0.04).value     # m; <= 0: use range_max
        self.tof_min_margin = p('tof_min_margin', 0.005).value      # m above range_min = valid
        self.tof_push_speed = p('tof_push_speed', 0.02).value       # m/s forward = "pushing"
        # [STUCK] (conservative on purpose)
        self.min_cmd_speed = p('min_cmd_speed', 0.08).value        # m/s forward
        self.stuck_time = p('stuck_time', 5.0).value                # s
        self.min_progress = p('min_progress', 0.05).value           # m
        self.odom_timeout = p('odom_timeout', 1.0).value            # s
        self.treat_lost_odom_as_stuck = p('treat_lost_odom_as_stuck', False).value
        # [NO PATH]
        self.inscribed_time = p('inscribed_time', 3.0).value        # s; <= 0 disables
        self.escape_probe = p('escape_probe', 0.2).value            # m ahead/behind to compare
        # Recovery motion (was: back up 0.2 m, then turn 1.57 rad left/right)
        self.backup_distance = p('backup_distance', 0.15).value     # m (was 0.2)
        self.backup_speed = p('backup_speed', 0.05).value           # m/s
        self.preempt_wait = p('preempt_wait', 1.0).value            # s
        self.settle_time = p('settle_time', 1.2).value              # s
        self.cooldown = p('cooldown', 10.0).value                   # s
        # Repeats at the same place
        self.repeat_radius = p('repeat_radius', 0.5).value          # m
        self.repeat_window = p('repeat_window', 120.0).value        # s
        self.max_backup_distance = p('max_backup_distance', 0.3).value  # m
        self.max_cooldown = p('max_cooldown', 40.0).value           # s
        # [ESCAPE] requests from frontier_explorer.py
        self.escape_topic = p('escape_topic', '/stuck_recovery/escape').value
        self.escape_turn_angle = p('escape_turn_angle', 0.8).value  # rad (~45 deg)
        self.escape_turn_speed = p('escape_turn_speed', 0.4).value  # rad/s

        # State
        self.phase = IDLE
        self.phase_end = None          # Time when the current phase ends
        self.move_sign = -1.0          # -1 = backwards, +1 = forwards
        self.move_distance = self.backup_distance
        self.this_cooldown = self.cooldown
        self.cmd_since = None          # Time since when we are pushed forward
        self.last_cmd_time = None
        self.last_forward_time = None  # last /cmd_vel with linear.x > tof_push_speed
        self.odom_samples = deque()    # (Time received, x, y) of visual odometry
        self.too_close = None          # (Time, distance m) of the latest too-close ToF reading
        self.inscribed_since = None    # Time since the robot centre is on an inscribed cell
        self.costmap = None
        self.explore_status = None
        self.paused_explore = False
        self.events = deque()          # (Time, x, y) of earlier recoveries
        self.escape_request = None     # (Time, reason) of the latest [ESCAPE] request
        self.turn_pending = 0.0        # rad still to turn after the move ([ESCAPE])
        self.turn_dir = 1.0

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.resume_pub = self.create_publisher(Bool, '/explore/resume', 10)
        self.create_subscription(Twist, self.cmd_vel_topic, self.on_cmd_vel, 10)
        self.create_subscription(Odometry, self.odom_topic, self.on_odom, 10)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(ExploreStatus, '/explore/status', self.on_status, latched)
        self.create_subscription(LaserScan, self.tof_topic, self.on_tof, 10)
        self.create_subscription(OccupancyGrid, self.costmap_topic, self.on_costmap, latched)
        self.create_subscription(String, self.escape_topic, self.on_escape, 10)

        self.nav_client = ActionClient(self, NavigateToPose, self.nav_action)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_timer(0.1, self.tick)  # 10 Hz: detection and recovery motion
        self.get_logger().info(
            f'Watching for a stuck robot while exploring: {self.tof_topic} closer than '
            f'{self.tof_too_close_m} m while driving forward, or {self.cmd_vel_topic} '
            f'forward > {self.min_cmd_speed} m/s for {self.stuck_time} s with '
            f'{self.odom_topic} moving < {self.min_progress} m, or standing on an '
            f'inscribed costmap cell for {self.inscribed_time} s')

    # ---------- inputs ----------

    def on_tof(self, msg):
        if self.phase != IDLE:
            return  # recovering or cooling down: do not re-fire on the same obstacle
        limit = self.tof_too_close_m if self.tof_too_close_m > 0.0 else msg.range_max
        lowest_valid = msg.range_min + self.tof_min_margin  # == range_min is a Gazebo clamp
        valid = [r for r in msg.ranges
                 if math.isfinite(r) and r > lowest_valid and r < limit]
        if valid:
            self.too_close = (self.now(), min(valid))

    def on_cmd_vel(self, msg):
        if self.phase != IDLE:
            return  # our own recovery commands, or Nav2 winding down
        now = self.now()
        self.last_cmd_time = now
        if msg.linear.x > self.tof_push_speed:
            self.last_forward_time = now
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

    def on_costmap(self, msg):
        self.costmap = msg

    def on_escape(self, msg):
        if self.phase != IDLE:
            self.get_logger().info(f'[ESCAPE] Request ignored, a recovery is already running: {msg.data}')
            return
        self.escape_request = (self.now(), msg.data)

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

    @staticmethod
    def yaw_of(pose):
        q = pose.pose.orientation
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def cost_at(self, x, y):
        """Published global costmap value at (x, y) in the map frame; None if unknown."""
        g = self.costmap
        if g is None or g.header.frame_id.lstrip('/') != self.map_frame:
            return None
        res = g.info.resolution
        col = int(math.floor((x - g.info.origin.position.x) / res))
        row = int(math.floor((y - g.info.origin.position.y) / res))
        if not (0 <= col < g.info.width and 0 <= row < g.info.height):
            return None
        v = g.data[row * g.info.width + col]
        return None if v < 0 else v

    def publish_cmd(self, linear=0.0, angular=0.0):
        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        self.cmd_pub.publish(cmd)

    def recent(self, t, now):
        return now - t < Duration(seconds=0.5)

    def pushing_without_moving(self, now):
        """True if pushed forward for stuck_time while visual odometry stays put."""
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
            return True, f'pushing into something (moved only {moved * 100:.1f} cm in {self.stuck_time:.0f} s)'
        return False, ''

    def on_inscribed_cell(self, now, pose):
        """True once the robot centre stayed on an inscribed/lethal cell for inscribed_time."""
        if self.inscribed_time <= 0.0 or pose is None:
            self.inscribed_since = None
            return False
        v = self.cost_at(pose.pose.position.x, pose.pose.position.y)
        if v is None or v < INSCRIBED_PUBLISHED:
            self.inscribed_since = None
            return False
        if self.inscribed_since is None:
            self.inscribed_since = now
        return now - self.inscribed_since >= Duration(seconds=self.inscribed_time)

    def turn_side(self, pose):
        """+1 (left) or -1 (right): the side with the lower costmap value 0.3 m away."""
        if pose is None:
            return 1.0
        yaw = self.yaw_of(pose)
        x, y = pose.pose.position.x, pose.pose.position.y
        left = self.cost_at(x - 0.3 * math.sin(yaw), y + 0.3 * math.cos(yaw))
        right = self.cost_at(x + 0.3 * math.sin(yaw), y - 0.3 * math.cos(yaw))
        left = 100 if left is None else left
        right = 100 if right is None else right
        return -1.0 if right < left else 1.0

    def escape_sign(self, pose):
        """+1 (forward) if the costmap is lower ahead than behind, else -1 (back)."""
        yaw = self.yaw_of(pose)
        x, y = pose.pose.position.x, pose.pose.position.y
        d = self.escape_probe
        ahead = self.cost_at(x + d * math.cos(yaw), y + d * math.sin(yaw))
        behind = self.cost_at(x - d * math.cos(yaw), y - d * math.sin(yaw))
        # Unknown counts as bad (cameras cannot see there); ties go backwards.
        ahead = 100 if ahead is None else ahead
        behind = 100 if behind is None else behind
        return 1.0 if ahead < behind else -1.0

    # ---------- main loop ----------

    def tick(self):
        now = self.now()
        if self.phase == IDLE:
            if not self.exploring():
                # Exploration finished or paused: leave the robot alone.
                self.cmd_since = None
                self.too_close = None
                self.inscribed_since = None
                self.escape_request = None
                return
            pose = self.robot_pose()
            too_close = (self.too_close is not None and self.recent(self.too_close[0], now)
                         and self.last_forward_time is not None
                         and self.recent(self.last_forward_time, now))
            stuck, reason = self.pushing_without_moving(now)
            if too_close:
                cm = self.too_close[1] * 100.0
                self.start_recovery(pose, '[TOO CLOSE]',
                                    f'Something is {cm:.1f} cm in front of the robot while driving',
                                    blacklist=True, sign=-1.0)
            elif stuck:
                self.start_recovery(pose, '[STUCK]', f'Robot {reason}', blacklist=True, sign=-1.0)
            elif self.on_inscribed_cell(now, pose):
                sign = self.escape_sign(pose)
                self.start_recovery(
                    pose, '[NO PATH]',
                    'Robot stands too close to a wall for Nav2 to plan from here',
                    blacklist=False, sign=sign)
            elif self.escape_request is not None:
                reason = self.escape_request[1]
                self.escape_request = None
                self.turn_dir = self.turn_side(pose)
                self.start_recovery(pose, '[ESCAPE]', f'Explorer asks for help ({reason})',
                                    blacklist=False, sign=-1.0)
                self.turn_pending = abs(self.escape_turn_angle)
                if self.turn_pending:
                    side = 'left' if self.turn_dir > 0 else 'right'
                    self.get_logger().warn(
                        f'[ESCAPE] After backing up: turning {math.degrees(self.turn_pending):.0f} deg {side}.')
        elif self.phase == PREEMPT:
            self.publish_cmd()  # keep the robot stopped while Nav2 lets go
            if now >= self.phase_end:
                if self.exploring():
                    self.resume_pub.publish(Bool(data=False))  # pause; cancels our goal too
                    self.paused_explore = True
                self.enter(SETTLE, self.settle_time)
        elif self.phase == SETTLE:
            self.publish_cmd()  # zeros only, until the move starts
            if now >= self.phase_end:
                self.enter(MOVE, self.move_distance / max(self.backup_speed, 1e-3))
        elif self.phase == MOVE and now >= self.phase_end and self.turn_pending:
            self.publish_cmd()  # stop, then turn ([ESCAPE] only)
            self.enter(TURN, self.turn_pending / max(abs(self.escape_turn_speed), 1e-3))
            self.turn_pending = 0.0
        elif self.phase == TURN:
            if now >= self.phase_end:
                self.publish_cmd()
                self.enter(MOVE, 0.0)  # finish like every other recovery (resume, cooldown)
            else:
                self.publish_cmd(angular=self.turn_dir * abs(self.escape_turn_speed))
        elif self.phase == MOVE:
            if now >= self.phase_end:
                self.publish_cmd()  # stop
                if self.paused_explore:
                    self.resume_pub.publish(Bool(data=True))
                    self.paused_explore = False
                    self.get_logger().info('[STUCK] Recovery done, exploration continues.')
                else:
                    self.get_logger().info('[STUCK] Recovery done.')
                self.enter(COOLDOWN, self.this_cooldown)
            else:
                self.publish_cmd(linear=self.move_sign * abs(self.backup_speed))
        elif self.phase == COOLDOWN:
            if now >= self.phase_end:
                self.cmd_since = None
                self.too_close = None
                self.inscribed_since = None
                self.phase = IDLE

    def enter(self, phase, seconds):
        self.phase = phase
        self.phase_end = self.now() + Duration(seconds=seconds)

    def count_repeats(self, pose, now):
        """Number of earlier recoveries near this pose within repeat_window."""
        window = Duration(seconds=self.repeat_window)
        while self.events and now - self.events[0][0] > window:
            self.events.popleft()
        if pose is None:
            return 0
        x, y = pose.pose.position.x, pose.pose.position.y
        n = sum(1 for _, ex, ey in self.events
                if math.hypot(ex - x, ey - y) < self.repeat_radius)
        self.events.append((now, x, y))
        return n

    def start_recovery(self, pose, tag, what, blacklist, sign):
        now = self.now()
        repeats = self.count_repeats(pose, now)
        self.move_distance = min(self.backup_distance * (1.0 + 0.5 * repeats),
                                 self.max_backup_distance)
        self.this_cooldown = min(self.cooldown * (2 ** repeats), self.max_cooldown)
        self.move_sign = sign
        where = (f'({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})'
                 if pose is not None else '(position unknown)')
        rep = f' (repeat {repeats} here)' if repeats else ''
        way = 'backing up' if sign < 0 else 'driving forward'
        then = 'choosing another frontier' if blacklist else 'then continuing'
        self.get_logger().warn(
            f'{tag}{rep} {what} at {where}: {way} {self.move_distance:.2f} m and {then}')
        self.cmd_since = None
        self.too_close = None
        self.inscribed_since = None
        self.publish_cmd()  # (a) stop now
        if blacklist:
            # (b) Replace explore_lite's goal with ours so Nav2 aborts it -> blacklisted.
            if pose is not None and self.nav_client.server_is_ready():
                goal = NavigateToPose.Goal()
                goal.pose = pose
                self.nav_client.send_goal_async(goal)
            else:
                self.get_logger().warn(
                    f'{tag} Could not replace the Nav2 goal (no map pose or Nav2 not ready); '
                    'explore_lite may try the same frontier again.')
            self.enter(PREEMPT, self.preempt_wait)
        else:
            self.enter(PREEMPT, 0.0)  # no goal swap: go straight to pausing explore


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
