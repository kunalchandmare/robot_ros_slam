#!/usr/bin/env python3
"""frontier_explorer.py - our own frontier exploration node (replaces explore_lite).

Started by launch.py with explore:=true explorer:=own (the default is still
explorer:=explore_lite). Parameters: config/frontier_explorer_params.yaml.
The frontier choice itself lives in frontier_core.py (plain NumPy, tested
offline by test/test_frontier_selection.py).

Same interfaces as explore_lite, so explore_map_saver.py and stuck_recovery.py
work unchanged:
  /explore/status  explore_lite_msgs/ExploreStatus, reliable + transient local:
                   "exploration_started" once the first look-around is done,
                   "exploration_paused" / "exploration_in_progress" on
                   /explore/resume, "exploration_complete" when no reachable
                   frontier is left.
  /explore/resume  std_msgs/Bool: False = cancel the goal and pause,
                   True = continue (also after "complete").
  navigate_to_pose nav2_msgs/action/NavigateToPose goals in the map frame.
  /explore/frontiers visualization_msgs/MarkerArray for RViz (usable frontier
                   cells green, current goal blue, blacklisted areas grey).

How it decides (one log line per decision, all prefixed [EXPLORER]):
  1. Look around first: until the costmap knows min_known_area_m2, turn on
     the spot (look_around_angle at look_around_speed) before the first goal.
     No status is published during that first turn, so stuck_recovery.py
     stays idle. The same happens (with "paused" status) if no frontier is
     left while the map is still small, at most max_look_arounds times.
  2. Every replan_period seconds frontier_core.select_frontiers() runs in a
     worker thread on the latest global costmap: only frontiers the robot can
     really reach through gaps of at least min_passage_width, ranked by path
     length minus a size bonus; the goal is an approach cell well away from
     walls, never the frontier centroid.
  3. Commit: the current goal is kept until Nav2 reports success or failure,
     the frontier is gone (seen), no progress (path length to the goal not
     shorter by progress_min within progress_timeout), or another frontier
     scores better by more than switch_margin metres.
  4. Failures: an aborted goal (planner or controller gave up, or
     stuck_recovery replaced it) blacklists the area around it
     (blacklist_radius) for blacklist_time seconds, doubling on every repeat
     there, permanently after blacklist_max_failures. A goal reached
     visit_limit times while its frontier is still there is blacklisted too.
     "Complete" is only reported when no frontier is left except permanently
     blacklisted ones (it waits for temporary entries to run out first), and
     only after a second selection confirms it.
  5. Escape: escape_after_aborts aborted goals in a row while the robot moved
     less than escape_motion metres means the controller refuses to move
     (e.g. "collision ahead" next to furniture). Then this node asks
     stuck_recovery.py (the only node that drives the robot backwards) for
     its [ESCAPE] recovery by publishing the reason on escape_topic
     (std_msgs/String). stuck_recovery pauses exploration, backs up, turns,
     and resumes it. Nothing here drives backwards, so the two never fight.
"""

import math
import threading

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time

import tf2_ros
from action_msgs.msg import GoalStatus
from explore_lite_msgs.msg import ExploreStatus
from geometry_msgs.msg import Point, PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool, ColorRGBA, String
from visualization_msgs.msg import Marker, MarkerArray

import frontier_core as fcore

# states
WAIT = 'wait'          # waiting for costmap, TF and Nav2
LOOK = 'look'          # turning on the spot
SELECT = 'select'      # choosing the next goal
NAV = 'nav'            # a goal is active
PAUSED = 'paused'      # /explore/resume False
DONE = 'done'          # exploration complete


class FrontierExplorer(Node):

    def __init__(self):
        super().__init__('frontier_explorer')
        p = lambda name, default: self.declare_parameter(name, default).value  # noqa: E731
        self.costmap_topic = p('costmap_topic', '/global_costmap/costmap')
        self.base_frame = p('robot_base_frame', 'base_footprint')
        self.nav_action = p('navigate_action', 'navigate_to_pose')
        self.cmd_vel_topic = p('cmd_vel_topic', '/cmd_vel')
        self.escape_topic = p('escape_topic', '/stuck_recovery/escape')
        self.visualize = bool(p('visualize', True))
        self.sp = fcore.SelectParams(
            min_passage_width=float(p('min_passage_width', 0.36)),
            comfortable_clearance=float(p('comfortable_clearance', 0.30)),
            narrow_penalty=float(p('narrow_penalty', 2.0)),
            min_frontier_size=float(p('min_frontier_size', 0.30)),
            approach_radius=float(p('approach_radius', 0.60)),
            approach_max_cost=int(p('approach_max_cost', 50)),
            free_check_radius=float(p('free_check_radius', 0.25)),
            min_free_ratio=float(p('min_free_ratio', 0.6)),
            size_gain=float(p('size_gain', 1.0)),
            size_cap=float(p('size_cap', 2.0)),
            start_search_radius=float(p('start_search_radius', 0.8)),
            ignore_near_robot=float(p('ignore_near_robot', 0.45)),
            blacklist_radius=float(p('blacklist_radius', 0.5)),
        )
        if bool(p('geofence_enabled', False)):
            self.sp.geofence = (float(p('geofence_xmin', -100.0)), float(p('geofence_xmax', 100.0)),
                                float(p('geofence_ymin', -100.0)), float(p('geofence_ymax', 100.0)))
        self.replan_period = float(p('replan_period', 3.0))
        self.progress_timeout = float(p('progress_timeout', 30.0))
        self.progress_min = float(p('progress_min', 0.10))
        self.switch_margin = float(p('switch_margin', 1.5))
        self.min_goal_distance = float(p('min_goal_distance', 0.25))
        self.min_known_area = float(p('min_known_area_m2', 10.0))
        self.look_speed = float(p('look_around_speed', 0.3))
        self.look_angle = float(p('look_around_angle', 6.28))
        self.look_settle = float(p('look_around_settle', 2.0))
        self.max_look_arounds = int(p('max_look_arounds', 2))
        self.blacklist_time = float(p('blacklist_time', 60.0))
        self.blacklist_max_failures = int(p('blacklist_max_failures', 3))
        self.visit_limit = int(p('visit_limit', 2))
        self.escape_after_aborts = int(p('escape_after_aborts', 2))
        self.escape_motion = float(p('escape_motion', 0.10))
        self.escape_cooldown = float(p('escape_cooldown', 30.0))

        self.state = WAIT
        self.started = False          # "exploration_started" published
        self.look_count = 0
        self.look_end = None
        self.look_resume = None
        self.costmap = None           # latest OccupancyGrid
        self.grid = None              # fcore.Grid of it
        self.blacklist = []           # dicts: x, y, fails, until (s), reason
        self.visits = []              # dicts: x, y, n (successful visits)
        self.goal = None              # current fcore.Candidate
        self.goal_handle = None
        self.goal_seq = 0
        self.goal_sent_at = None
        self.best_dist = math.inf     # shortest path length to the goal seen so far
        self.last_progress = None
        self.abort_streak = []        # (time s, x, y) of aborts in a row
        self.last_escape = -1e9
        self.goals_sent = 0
        self.worker = None            # threading.Thread running the selection
        self.result = None            # (stamp s, Selection, robot xy)
        self.result_lock = threading.Lock()
        self.last_select_start = -1e9
        self.select_wait = 0.0        # s between selections while in SELECT
        self.empty_count = 0          # selections in a row without a usable frontier
        self.dropped_now = 0

        latched = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(ExploreStatus, '/explore/status', latched)
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.escape_pub = self.create_publisher(String, self.escape_topic, 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/explore/frontiers', 10)
        costmap_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, self.costmap_topic, self.on_costmap, costmap_qos)
        self.create_subscription(Bool, '/explore/resume', self.on_resume, 10)
        self.nav_client = ActionClient(self, NavigateToPose, self.nav_action)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.create_timer(0.1, self.tick)
        self.log(f'Waiting for {self.costmap_topic}, TF map -> {self.base_frame} and Nav2 '
                 f'({self.nav_action}). Gaps must be >= {self.sp.min_passage_width:.2f} m, '
                 f'look around until {self.min_known_area:.0f} m2 known.')

    # ------------------------------------------------------------ helpers
    def log(self, text):
        self.get_logger().info('[EXPLORER] ' + text)

    def warn(self, text):
        self.get_logger().warn('[EXPLORER] ' + text)

    def now_s(self):
        return self.get_clock().now().nanoseconds / 1e9

    def publish_status(self, status):
        msg = ExploreStatus()
        msg.status = status
        self.status_pub.publish(msg)
        self.log(f'status: {status}')

    def robot_xy(self):
        frame = self.costmap.header.frame_id.lstrip('/') if self.costmap else 'map'
        try:
            t = self.tf_buffer.lookup_transform(frame, self.base_frame, Time())
        except tf2_ros.TransformException:
            return None
        return (t.transform.translation.x, t.transform.translation.y)

    def active_blacklist(self):
        now = self.now_s()
        return [(b['x'], b['y']) for b in self.blacklist if b['until'] > now]

    def add_blacklist(self, x, y, reason):
        now = self.now_s()
        for b in self.blacklist:
            if math.hypot(b['x'] - x, b['y'] - y) < self.sp.blacklist_radius:
                b['fails'] += 1
                break
        else:
            b = {'x': x, 'y': y, 'fails': 1}
            self.blacklist.append(b)
        if b['fails'] >= self.blacklist_max_failures:
            b['until'] = math.inf
            how = 'for good'
        else:
            secs = self.blacklist_time * 2 ** (b['fails'] - 1)
            b['until'] = now + secs
            how = f'for {secs:.0f} s'
        b['reason'] = reason
        self.warn(f'Blacklisted area ({x:.2f}, {y:.2f}) r={self.sp.blacklist_radius:.2f} m {how} '
                  f'(failure {b["fails"]} there): {reason}')

    def known_area(self):
        return fcore.known_area_m2(self.grid) if self.grid is not None else 0.0

    # ------------------------------------------------------------ inputs
    def on_costmap(self, msg):
        self.costmap = msg
        data = np.array(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width)
        self.grid = fcore.Grid(data, msg.info.resolution,
                               msg.info.origin.position.x, msg.info.origin.position.y)

    def on_resume(self, msg):
        if msg.data:
            if self.state in (PAUSED, DONE):
                self.log('Resume requested: exploring again.')
                self.publish_status(ExploreStatus.EXPLORATION_IN_PROGRESS)
                self.started = True
                self.state = SELECT
                self.goal = None
            else:
                self.log(f'Resume requested while {self.state}: nothing to do.')
        else:
            if self.state in (PAUSED, DONE):
                return
            self.log('Pause requested: cancelling the current goal.')
            if self.state == LOOK:
                self.cmd_pub.publish(Twist())
            self.cancel_goal()
            self.state = PAUSED
            self.publish_status(ExploreStatus.EXPLORATION_PAUSED)

    # ------------------------------------------------------------ main loop
    def tick(self):
        now = self.now_s()
        if self.state == WAIT:
            if self.grid is None or self.robot_xy() is None or not self.nav_client.server_is_ready():
                return
            if self.known_area() < self.min_known_area and self.max_look_arounds > 0:
                self.start_look(f'map knows only {self.known_area():.1f} m2 '
                                f'(< {self.min_known_area:.0f} m2) before the first goal')
            else:
                self.begin_exploring()
        elif self.state == LOOK:
            if now < self.look_end:
                cmd = Twist()
                cmd.angular.z = abs(self.look_speed)
                self.cmd_pub.publish(cmd)
            elif now < self.look_resume:
                self.cmd_pub.publish(Twist())
            else:
                self.log(f'Look-around done, {self.known_area():.1f} m2 known now.')
                if not self.started:
                    self.begin_exploring()
                else:
                    self.publish_status(ExploreStatus.EXPLORATION_IN_PROGRESS)
                    self.state = SELECT
        elif self.state in (SELECT, NAV):
            if self.worker is None and now - self.last_select_start >= (
                    self.select_wait if self.state == SELECT else self.replan_period):
                self.start_selection()
            with self.result_lock:
                res, self.result = self.result, None
            if res is not None:
                self.on_selection(*res)

    def begin_exploring(self):
        self.started = True
        self.publish_status(ExploreStatus.EXPLORATION_STARTED)
        self.state = SELECT

    def start_look(self, reason):
        self.look_count += 1
        turn_s = abs(self.look_angle) / max(abs(self.look_speed), 1e-3)
        self.log(f'Look-around {self.look_count} of {self.max_look_arounds}: {reason}; '
                 f'turning {math.degrees(self.look_angle):.0f} deg ({turn_s:.0f} s).')
        if self.started:
            self.publish_status(ExploreStatus.EXPLORATION_PAUSED)  # stuck_recovery stays idle
        self.look_end = self.now_s() + turn_s
        self.look_resume = self.look_end + self.look_settle
        self.state = LOOK

    # ------------------------------------------------------------ selection
    def start_selection(self):
        xy = self.robot_xy()
        if xy is None or self.grid is None:
            return
        grid = self.grid
        blacklist = self.active_blacklist()
        self.last_select_start = self.now_s()

        def work():
            try:
                sel = fcore.select_frontiers(grid, xy, self.sp, blacklist)
            except Exception as error:  # never kill the node
                sel = fcore.Selection(note=f'selection failed: {error}')
            with self.result_lock:
                self.result = (self.now_s(), sel, xy, grid)
            self.worker = None

        self.worker = threading.Thread(target=work, daemon=True)
        self.worker.start()

    def visited_too_often(self, cand):
        for v in self.visits:
            if math.hypot(v['x'] - cand.goal[0], v['y'] - cand.goal[1]) < self.sp.blacklist_radius:
                return v['n'] >= self.visit_limit
        return False

    def reset_visits(self, cand):
        for v in self.visits:
            if math.hypot(v['x'] - cand.goal[0], v['y'] - cand.goal[1]) < self.sp.blacklist_radius:
                v['n'] = 0

    def usable(self, sel, xy):
        out = []
        self.dropped_now = 0  # dropped here because of visits (counts like blacklisted)
        for c in sel.candidates:
            if math.hypot(c.goal[0] - xy[0], c.goal[1] - xy[1]) < self.min_goal_distance:
                # Standing on its approach cell and the frontier is still there:
                # count it as a visit (blacklisted after visit_limit), skip it now.
                self.record_visit(c)
                if not self.visited_too_often(c):
                    continue
            if self.visited_too_often(c):
                self.add_blacklist(c.goal[0], c.goal[1],
                                   f'reached {self.visit_limit} times but the frontier is still there')
                self.reset_visits(c)
                self.dropped_now += 1
                continue
            out.append(c)
        return out

    def on_selection(self, stamp, sel, xy, grid):
        if self.state not in (SELECT, NAV):
            return  # paused or done meanwhile
        cands = self.usable(sel, xy)
        if sel.note:
            self.log('Selection note: ' + sel.note)
        self.publish_markers(cands, grid)
        if sel.start_cell is None:
            # Robot not near any drivable cell (or the selection failed): never
            # call that "complete". stuck_recovery's [NO PATH] moves it off a wall.
            self.warn(f'Cannot plan from the robot position ({sel.note}); '
                      f'trying again in {self.replan_period:.0f} s.')
            self.select_wait = self.replan_period
            return
        if self.state == SELECT:
            if cands:
                self.empty_count = 0
                self.select_wait = 0.0
                self.send_goal(cands[0], sel, cands, 'best reachable frontier')
            else:
                self.no_frontiers(sel)
            return
        # NAV: keep the goal unless there is a reason to change
        cur = self.goal
        match = None
        for c in cands:
            if (math.hypot(c.goal[0] - cur.goal[0], c.goal[1] - cur.goal[1]) < self.sp.blacklist_radius
                    or math.hypot(c.middle[0] - cur.middle[0], c.middle[1] - cur.middle[1])
                    < self.sp.blacklist_radius):
                match = c
                break
        now = self.now_s()
        if sel.dist is not None:
            r, c = grid.to_cell(*cur.goal)
            d = float(sel.dist[r, c]) if grid.inside(r, c) else math.inf
            if d < self.best_dist - self.progress_min:
                self.best_dist = d
                self.last_progress = now
        if match is None:
            if cands:
                self.send_goal(cands[0], sel, cands,
                               'current frontier is explored or no longer reachable')
            else:
                self.cancel_goal()
                self.state = SELECT
                self.no_frontiers(sel)
            return
        if now - self.last_progress > self.progress_timeout:
            self.add_blacklist(cur.goal[0], cur.goal[1],
                               f'no progress for {self.progress_timeout:.0f} s '
                               f'(still {self.best_dist:.2f} m to go)')
            rest = [c for c in cands if c is not match]
            if rest:
                self.send_goal(rest[0], sel, rest, 'previous goal made no progress')
            else:
                self.cancel_goal()
                self.state = SELECT
            return
        best = cands[0]
        if best is not match and best.score + self.switch_margin < match.score:
            self.send_goal(best, sel, cands,
                           f'clearly better: score {best.score:.2f} vs {match.score:.2f} '
                           f'of the current goal (margin {self.switch_margin:.1f})')

    def no_frontiers(self, sel):
        # Confirm on a second costmap before giving up (a costmap reset or
        # resize can briefly hide every frontier).
        self.empty_count += 1
        if self.empty_count < 2:
            self.log(f'No reachable frontier ({sel.summary()}); checking again in 2 s.')
            self.select_wait = 2.0
            self.state = SELECT
            return
        self.empty_count = 0
        self.select_wait = 0.0
        now = self.now_s()
        temporary = [bl['until'] for bl in self.blacklist if now < bl['until'] < math.inf]
        if (sel.n_blacklisted > 0 or self.dropped_now > 0) and temporary:
            # Only blacklisted-for-now frontiers left: try them again when the
            # first blacklist entry runs out (repeats become permanent, so this ends).
            wait = min(temporary) - now + 0.5
            self.log(f'Only temporarily blacklisted frontiers left ({sel.summary()}); '
                     f'trying again in {wait:.0f} s.')
            self.select_wait = wait
            self.state = SELECT
            return
        area = self.known_area()
        if area < self.min_known_area and self.look_count < self.max_look_arounds:
            self.start_look(f'no reachable frontier ({sel.summary()}) and only {area:.1f} m2 known')
            return
        self.log(f'Exploration complete: no reachable frontier left ({sel.summary()}), '
                 f'{area:.1f} m2 known, {self.goals_sent} goals sent.')
        self.state = DONE
        self.goal = None
        self.publish_status(ExploreStatus.EXPLORATION_COMPLETE)

    # ------------------------------------------------------------ goals
    def send_goal(self, cand, sel, cands, reason):
        self.goal = cand
        self.goal_seq += 1
        self.goals_sent += 1
        seq = self.goal_seq
        msg = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = self.costmap.header.frame_id or 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y = cand.goal
        yaw = math.atan2(cand.middle[1] - cand.goal[1], cand.middle[0] - cand.goal[0])
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        msg.pose = pose
        self.best_dist = cand.path_m
        self.last_progress = self.now_s()
        self.goal_sent_at = self.now_s()
        others = ', '.join(f'({c.middle[0]:.1f},{c.middle[1]:.1f}) {c.score:.2f}'
                           for c in cands[1:4])
        self.log(f'Goal #{self.goals_sent}: frontier at ({cand.middle[0]:.2f}, {cand.middle[1]:.2f}) '
                 f'size {cand.size_m:.2f} m, approach goal ({cand.goal[0]:.2f}, {cand.goal[1]:.2f}), '
                 f'path {cand.path_m:.2f} m, score {cand.score:.2f}. Why: {reason}. '
                 f'[{sel.summary()}] next best: {others or "none"}')
        future = self.nav_client.send_goal_async(msg)
        future.add_done_callback(lambda f, s=seq: self.on_goal_response(f, s))
        self.state = NAV

    def on_goal_response(self, future, seq):
        handle = future.result()
        if seq != self.goal_seq:
            return  # an older goal; Nav2 preempted it with the newer one
        if handle is None or not handle.accepted:
            self.warn('Nav2 rejected the goal.')
            self.goal_failed('Nav2 rejected the goal')
            return
        self.goal_handle = handle
        handle.get_result_async().add_done_callback(lambda f, s=seq: self.on_result(f, s))

    def on_result(self, future, seq):
        if seq != self.goal_seq or self.state != NAV:
            return  # stale (preempted by our own newer goal) or paused
        status = future.result().status
        cur = self.goal
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.log(f'Reached the approach goal ({cur.goal[0]:.2f}, {cur.goal[1]:.2f}) '
                     f'in {self.now_s() - self.goal_sent_at:.0f} s.')
            self.record_visit(cur)
            self.abort_streak = []
            self.goal = None
            self.state = SELECT
        elif status == GoalStatus.STATUS_ABORTED:
            self.goal_failed('Nav2 aborted the goal (planner/controller gave up, '
                             'or stuck_recovery replaced it)')
        elif status == GoalStatus.STATUS_CANCELED:
            self.log('Goal canceled.')
            self.goal = None
            self.state = SELECT
        else:
            self.goal_failed(f'Nav2 ended the goal with status {status}')

    def goal_failed(self, reason):
        cur = self.goal
        if cur is not None:
            self.add_blacklist(cur.goal[0], cur.goal[1], reason)
        xy = self.robot_xy()
        if xy is not None:
            self.abort_streak.append((self.now_s(), xy[0], xy[1]))
        self.goal = None
        self.goal_handle = None
        self.state = SELECT
        self.check_escape()

    def record_visit(self, cand):
        for v in self.visits:
            if math.hypot(v['x'] - cand.goal[0], v['y'] - cand.goal[1]) < self.sp.blacklist_radius:
                v['n'] += 1
                return
        self.visits.append({'x': cand.goal[0], 'y': cand.goal[1], 'n': 1})

    def cancel_goal(self):
        self.goal_seq += 1  # results of the old goal are ignored from now on
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        self.goal_handle = None
        self.goal = None

    # ------------------------------------------------------------ escape
    def check_escape(self):
        n = self.escape_after_aborts
        if n <= 0 or len(self.abort_streak) < n:
            return
        recent = self.abort_streak[-n:]
        moved = max(math.hypot(x - recent[0][1], y - recent[0][2]) for _, x, y in recent)
        if moved >= self.escape_motion:
            return
        now = self.now_s()
        if now - self.last_escape < self.escape_cooldown:
            self.log(f'{n} aborts in a row without moving, but the last escape was only '
                     f'{now - self.last_escape:.0f} s ago; waiting.')
            return
        reason = (f'{n} goals aborted in a row while the robot moved only {moved * 100:.0f} cm '
                  '(controller refuses to drive, e.g. collision ahead)')
        if self.escape_pub.get_subscription_count() == 0:
            self.warn(f'{reason}, but nobody listens on {self.escape_topic} '
                      '(stuck_recovery.py not running?). No escape.')
            return
        self.last_escape = now
        self.abort_streak = []
        self.warn(f'Asking stuck_recovery for an escape: {reason}')
        self.escape_pub.publish(String(data=reason))

    # ------------------------------------------------------------ RViz
    def publish_markers(self, cands, grid):
        if not self.visualize:
            return
        frame = self.costmap.header.frame_id or 'map'
        stamp = self.get_clock().now().to_msg()
        arr = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)
        pts = Marker()
        pts.header.frame_id, pts.header.stamp = frame, stamp
        pts.ns, pts.id, pts.type = 'frontiers', 0, Marker.POINTS
        pts.scale.x = pts.scale.y = grid.resolution * 2
        pts.color = ColorRGBA(r=0.0, g=0.9, b=0.0, a=1.0)
        pts.pose.orientation.w = 1.0
        for c in cands:
            for r, col in c.cells[::2]:
                x, y = grid.to_world(int(r), int(col))
                pts.points.append(Point(x=x, y=y, z=0.05))
        arr.markers.append(pts)
        nid = 1
        targets = [(c.goal, ColorRGBA(r=0.6, g=0.9, b=0.6, a=0.8), 0.08) for c in cands]
        if self.goal is not None:
            targets.append((self.goal.goal, ColorRGBA(r=0.1, g=0.3, b=1.0, a=1.0), 0.15))
        for (x, y), color, size in targets:
            m = Marker()
            m.header.frame_id, m.header.stamp = frame, stamp
            m.ns, m.id, m.type = 'goals', nid, Marker.SPHERE
            m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, 0.1
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = size
            m.color = color
            arr.markers.append(m)
            nid += 1
        for x, y in self.active_blacklist():
            m = Marker()
            m.header.frame_id, m.header.stamp = frame, stamp
            m.ns, m.id, m.type = 'blacklist', nid, Marker.CYLINDER
            m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, 0.01
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = 2 * self.sp.blacklist_radius
            m.scale.z = 0.02
            m.color = ColorRGBA(r=0.5, g=0.5, b=0.5, a=0.4)
            arr.markers.append(m)
            nid += 1
        self.marker_pub.publish(arr)


def main():
    rclpy.init()
    node = FrontierExplorer()
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
