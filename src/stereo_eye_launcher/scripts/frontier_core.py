#!/usr/bin/env python3
"""frontier_core.py - frontier selection for frontier_explorer.py (no ROS needed).

Plain Python 3 + NumPy, so the same code runs inside the ROS node and in the
offline test (test/test_frontier_selection.py) on any computer.

Input: Nav2's global costmap as published on /global_costmap/costmap
(nav_msgs/OccupancyGrid values): -1 unknown, 0 free, 1-98 inflated
(the closer to a wall the higher), 99 inscribed (the robot circle would touch
a wall), 100 lethal (a wall).

What select_frontiers() does, in order (lessons from the 2026-11-24
regression, where explore_lite sent the robot to "frontiers" outside the
building through holes in the mapped walls):

 1. Where can the robot centre go?  A cell is DRIVABLE if it is known, below
    "inscribed" (value < 99) AND at least min_passage_width / 2 away from the
    nearest wall cell (value 100). With min_passage_width 0.36 m (robot
    0.256 m + 5 cm per side) a gap must be at least 0.36 m wide to count as a
    way through. A 0.25 m hole in a wall is never crossed.
 2. Path length from the robot to every drivable cell: Dijkstra over drivable
    cells, 8 neighbours. Steps through cells with less than
    comfortable_clearance (0.30 m) to the nearest wall cost
    (1 + narrow_penalty) times more, so wide doorways are preferred over
    barely-wide-enough gaps. Cells the robot cannot reach get no distance.
 3. Frontier cells: unknown cells with a known free neighbour (4-connected),
    grouped into 8-connected clusters. Clusters shorter than
    min_frontier_size (0.3 m) are dropped.
 4. For each cluster an APPROACH cell (the Nav2 goal; never the centroid,
    which can lie inside a wall or outside the building): a REACHABLE cell
    within approach_radius (0.6 m) of the cluster, with costmap value below
    approach_max_cost (50 = Nav2 cost ~128, well away from walls), whose
    surroundings (radius 0.25 m) are mostly known free space (min_free_ratio)
    - isolated free specks seen through a wall do not qualify. The one closest
    to the frontier wins (ties: shorter path), provided the straight line from
    it to the frontier crosses no wall cell. No such cell = UNREACHABLE.
 5. Unknown patches lying completely within ignore_near_robot of the robot
    (the cameras' blind zone, cells under the robot) are skipped.
    Blacklisted areas (radius around a failed goal) and the optional geofence
    remove candidates.
 6. Score = path length (m) - size_gain * min(frontier length, size_cap) (m).
    Lowest score first.

Everything returned carries numbers for the log line (path length, size,
score, why others were skipped).
"""

import heapq
import math
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np

UNKNOWN = -1
INSCRIBED = 99   # published value of nav2 cost 253
LETHAL = 100     # published value of nav2 cost 254


@dataclass
class SelectParams:
    min_passage_width: float = 0.36      # m; robot 0.256 m + 2 x 0.05 m
    comfortable_clearance: float = 0.30  # m; below this the step costs more
    narrow_penalty: float = 2.0          # extra cost factor in narrow places
    min_frontier_size: float = 0.30      # m of frontier length
    approach_radius: float = 0.60        # m from the frontier to the goal cell
    approach_max_cost: int = 50          # published 0-100 scale (~Nav2 128)
    free_check_radius: float = 0.25      # m around the goal cell
    min_free_ratio: float = 0.6          # share of known free cells there
    size_gain: float = 1.0               # m of path worth 1 m of frontier
    size_cap: float = 2.0                # m; longer frontiers get no extra bonus
    start_search_radius: float = 0.8     # m; robot on a non-drivable cell
    ignore_near_robot: float = 0.45      # m; unknown patches this close (cameras' blind
                                         # zone, cells under the robot) are not frontiers
    blacklist_radius: float = 0.5        # m
    geofence: Tuple[float, ...] = ()     # (xmin, xmax, ymin, ymax) map frame; () = off


@dataclass
class Grid:
    data: np.ndarray        # int16 (height, width), row 0 = lowest y
    resolution: float
    origin_x: float
    origin_y: float

    def to_cell(self, x, y):
        return (int(math.floor((y - self.origin_y) / self.resolution)),
                int(math.floor((x - self.origin_x) / self.resolution)))

    def to_world(self, r, c):
        return (self.origin_x + (c + 0.5) * self.resolution,
                self.origin_y + (r + 0.5) * self.resolution)

    def inside(self, r, c):
        return 0 <= r < self.data.shape[0] and 0 <= c < self.data.shape[1]


@dataclass
class Candidate:
    goal: Tuple[float, float]          # approach cell (map frame), the Nav2 goal
    goal_cell: Tuple[int, int]
    middle: Tuple[float, float]        # frontier cell closest to the goal
    size_m: float
    path_m: float
    score: float
    n_cells: int
    cells: Optional[np.ndarray] = None  # (n, 2) row/col of the frontier cells


@dataclass
class Selection:
    candidates: List[Candidate] = field(default_factory=list)  # best first
    n_clusters: int = 0
    n_small: int = 0
    n_unreachable: int = 0
    n_blacklisted: int = 0
    n_geofenced: int = 0
    n_at_robot: int = 0
    start_cell: Optional[Tuple[int, int]] = None
    note: str = ''
    dist: Optional[np.ndarray] = None   # path length (m) per cell, inf = unreachable

    def summary(self):
        return (f'{self.n_clusters} frontier groups: {len(self.candidates)} usable, '
                f'{self.n_small} too small, {self.n_unreachable} unreachable, '
                f'{self.n_blacklisted} blacklisted, {self.n_geofenced} outside geofence, '
                f'{self.n_at_robot} at the robot')


# ---------------------------------------------------------------- helpers

def _shift(a, dr, dc, fill):
    """a shifted so that out[r, c] = a[r - dr, c - dc] (fill outside)."""
    out = np.full_like(a, fill)
    h, w = a.shape
    rs, re = max(dr, 0), h + min(dr, 0)
    cs, ce = max(dc, 0), w + min(dc, 0)
    out[rs:re, cs:ce] = a[rs - dr:re - dr, cs - dc:ce - dc]
    return out


def distance_to_mask(mask, max_dist_cells):
    """Euclidean distance (cells) to the nearest True cell, capped at max_dist_cells + 1."""
    cap = float(max_dist_cells) + 1.0
    dist = np.full(mask.shape, cap, dtype=np.float32)
    dist[mask] = 0.0
    if not mask.any():
        return dist
    m = int(math.ceil(max_dist_cells))
    offsets = sorted(((math.hypot(dr, dc), dr, dc)
                      for dr in range(-m, m + 1) for dc in range(-m, m + 1)
                      if 0 < math.hypot(dr, dc) <= max_dist_cells))
    for d, dr, dc in offsets:
        near = _shift(mask, dr, dc, False)
        upd = near & (dist > d)
        dist[upd] = d
    return dist


def disk_mean(values, radius_cells):
    """Mean of values over a disk around every cell (outside the grid counts as 0)."""
    r = int(math.floor(radius_cells))
    acc = np.zeros(values.shape, dtype=np.float32)
    n = 0
    for dr in range(-r, r + 1):
        for dc in range(-r, r + 1):
            if dr * dr + dc * dc <= radius_cells * radius_cells:
                acc += _shift(values, dr, dc, 0.0)
                n += 1
    return acc / max(n, 1)


def dijkstra(drivable, step_factor, start, resolution):
    """Path length (m) from start over drivable cells, 8-connected; inf = unreachable."""
    h, w = drivable.shape
    flat_ok = drivable.ravel()
    flat_f = step_factor.ravel()
    dist = np.full(h * w, np.inf)
    s = start[0] * w + start[1]
    dist[s] = 0.0
    heap = [(0.0, s)]
    d1, d2 = resolution, resolution * math.sqrt(2.0)
    nbrs = ((-1, 0, d1), (1, 0, d1), (0, -1, d1), (0, 1, d1),
            (-1, -1, d2), (-1, 1, d2), (1, -1, d2), (1, 1, d2))
    dist_l = dist  # local names are faster in the loop
    while heap:
        d, i = heapq.heappop(heap)
        if d > dist_l[i]:
            continue
        r, c = divmod(i, w)
        for dr, dc, step in nbrs:
            rr, cc = r + dr, c + dc
            if rr < 0 or rr >= h or cc < 0 or cc >= w:
                continue
            j = rr * w + cc
            if not flat_ok[j]:
                continue
            if dr and dc and not (flat_ok[r * w + cc] and flat_ok[rr * w + c]):
                continue  # no diagonal squeeze between two blocked cells
            nd = d + step * 0.5 * (flat_f[i] + flat_f[j])
            if nd < dist_l[j]:
                dist_l[j] = nd
                heapq.heappush(heap, (nd, j))
    return dist.reshape(h, w)


def frontier_clusters(data):
    """List of (n, 2) int arrays: 8-connected groups of unknown cells next to free space."""
    unknown = data == UNKNOWN
    free = (data >= 0) & (data < INSCRIBED)
    near_free = (_shift(free, 1, 0, False) | _shift(free, -1, 0, False)
                 | _shift(free, 0, 1, False) | _shift(free, 0, -1, False))
    fmask = unknown & near_free
    h, w = data.shape
    seen = np.zeros_like(fmask)
    clusters = []
    for r0, c0 in zip(*np.nonzero(fmask)):
        if seen[r0, c0]:
            continue
        seen[r0, c0] = True
        stack = [(r0, c0)]
        cells = []
        while stack:
            r, c = stack.pop()
            cells.append((r, c))
            for dr in (-1, 0, 1):
                for dc in (-1, 0, 1):
                    rr, cc = r + dr, c + dc
                    if 0 <= rr < h and 0 <= cc < w and fmask[rr, cc] and not seen[rr, cc]:
                        seen[rr, cc] = True
                        stack.append((rr, cc))
        clusters.append(np.array(cells, dtype=np.int32))
    return clusters


def line_is_clear(wall, r0, c0, r1, c1):
    """True if the straight line between two cells crosses no wall cell."""
    n = max(abs(r1 - r0), abs(c1 - c0))
    if n == 0:
        return not wall[r0, c0]
    t = np.linspace(0.0, 1.0, n + 1)
    rr = np.round(r0 + (r1 - r0) * t).astype(np.int64)
    cc = np.round(c0 + (c1 - c0) * t).astype(np.int64)
    return not wall[rr, cc].any()


def in_blacklist(x, y, blacklist, radius):
    return any(math.hypot(x - bx, y - by) < radius for bx, by in blacklist)


def in_geofence(x, y, fence):
    if not fence:
        return True
    xmin, xmax, ymin, ymax = fence
    return xmin <= x <= xmax and ymin <= y <= ymax


# ---------------------------------------------------------------- main entry

def select_frontiers(grid, robot_xy, params=None, blacklist=()):
    """Rank reachable frontiers. blacklist: iterable of (x, y) areas to avoid."""
    p = params or SelectParams()
    data = grid.data.astype(np.int16)
    res = grid.resolution
    sel = Selection()

    wall = data >= LETHAL
    clear_need = 0.5 * p.min_passage_width / res
    comfy = p.comfortable_clearance / res
    wall_dist = distance_to_mask(wall, max(clear_need, comfy) + 1.0)
    known_ok = (data >= 0) & (data < INSCRIBED)
    drivable = known_ok & (wall_dist >= clear_need)
    step_factor = np.where(wall_dist < comfy, 1.0 + p.narrow_penalty, 1.0).astype(np.float32)

    # start cell: the robot's, or the nearest drivable one within start_search_radius
    r0, c0 = grid.to_cell(*robot_xy)
    start = None
    if grid.inside(r0, c0) and drivable[r0, c0]:
        start = (r0, c0)
    else:
        rad = int(p.start_search_radius / res)
        best = None
        for dr in range(-rad, rad + 1):
            for dc in range(-rad, rad + 1):
                rr, cc = r0 + dr, c0 + dc
                if grid.inside(rr, cc) and drivable[rr, cc]:
                    d = dr * dr + dc * dc
                    if d <= rad * rad and (best is None or d < best[0]):
                        best = (d, rr, cc)
        if best is not None:
            start = (best[1], best[2])
            sel.note = (f'robot cell not drivable, starting from the nearest drivable cell '
                        f'{math.sqrt(best[0]) * res:.2f} m away')
    if start is None:
        sel.note = 'robot is not near any drivable cell (no path possible from here)'
        return sel
    sel.start_cell = start

    dist = dijkstra(drivable, step_factor, start, res)
    sel.dist = dist
    reachable = np.isfinite(dist)

    free = known_ok.astype(np.float32)
    free_ratio = disk_mean(free, p.free_check_radius / res)
    goal_ok = reachable & (data < p.approach_max_cost) & (data >= 0) & (free_ratio >= p.min_free_ratio)

    # Space the robot can SEE from where it can drive: the reachable drivable
    # cells, grown by at most min_passage_width / 2 through known non-wall cells
    # (that reaches the free cells next to walls, but never through a wall:
    # the walk stops at the first wall cell). Only frontier cells touching this
    # space count. A frontier behind a wall or behind a too-narrow gap does not.
    seen_from_reach = reachable.copy()
    see_through = (data >= 0) & (data < LETHAL)
    for _ in range(int(math.ceil(clear_need))):
        grown = (_shift(seen_from_reach, 1, 0, False) | _shift(seen_from_reach, -1, 0, False)
                 | _shift(seen_from_reach, 0, 1, False) | _shift(seen_from_reach, 0, -1, False))
        seen_from_reach |= grown & see_through
    touches = (_shift(seen_from_reach, 1, 0, False) | _shift(seen_from_reach, -1, 0, False)
               | _shift(seen_from_reach, 0, 1, False) | _shift(seen_from_reach, 0, -1, False))

    clusters = frontier_clusters(data)
    sel.n_clusters = len(clusters)
    rad = p.approach_radius / res
    h, w = data.shape
    near_r = p.ignore_near_robot / res
    for cells in clusters:
        if near_r > 0 and np.all(np.hypot(cells[:, 0] - r0, cells[:, 1] - c0) <= near_r):
            sel.n_at_robot += 1  # blind spot around / under the robot, not a frontier
            continue
        cells = cells[touches[cells[:, 0], cells[:, 1]]]  # the part visible from our side
        if len(cells) == 0:
            sel.n_unreachable += 1
            continue
        size_m = len(cells) * res
        if size_m < p.min_frontier_size:
            sel.n_small += 1
            continue
        rmin, cmin = cells.min(axis=0)
        rmax, cmax = cells.max(axis=0)
        ri = int(math.ceil(rad))
        r1, r2 = max(rmin - ri, 0), min(rmax + ri + 1, h)
        c1, c2 = max(cmin - ri, 0), min(cmax + ri + 1, w)
        sub_ok = goal_ok[r1:r2, c1:c2]
        cand = np.argwhere(sub_ok)
        if len(cand) == 0:
            sel.n_unreachable += 1
            continue
        cand = cand + np.array([r1, c1])
        # distance (cells) from each candidate to the nearest frontier cell
        step = max(1, len(cells) // 400)  # cap the work for very long frontiers
        fc = cells[::step].astype(np.float32)
        dmin = np.full(len(cand), np.inf, dtype=np.float32)
        nearest = np.zeros(len(cand), dtype=np.int64)
        for k0 in range(0, len(fc), 200):
            blk = fc[k0:k0 + 200]
            d = np.hypot(cand[:, None, 0] - blk[None, :, 0], cand[:, None, 1] - blk[None, :, 1])
            j = d.argmin(axis=1)
            dj = d[np.arange(len(cand)), j]
            better = dj < dmin
            dmin[better] = dj[better]
            nearest[better] = j[better] + k0
        within = dmin <= rad
        if not within.any():
            sel.n_unreachable += 1
            continue
        cand, dmin, nearest = cand[within], dmin[within], nearest[within]
        paths = dist[cand[:, 0], cand[:, 1]]
        order = np.lexsort((paths, np.round(dmin, 1)))  # closest to frontier, then shortest path
        # first candidate with a clear line of sight (no wall cell) to its frontier cell
        k = None
        for kk in order[:200]:
            mr, mc = fc[nearest[kk]]
            if line_is_clear(wall, int(cand[kk, 0]), int(cand[kk, 1]), int(mr), int(mc)):
                k = kk
                break
        if k is None:
            sel.n_unreachable += 1
            continue
        gr, gc = int(cand[k, 0]), int(cand[k, 1])
        gx, gy = grid.to_world(gr, gc)
        mr, mc = fc[nearest[k]]
        mx, my = grid.to_world(int(mr), int(mc))
        if not (in_geofence(gx, gy, p.geofence) and in_geofence(mx, my, p.geofence)):
            sel.n_geofenced += 1
            continue
        if in_blacklist(gx, gy, blacklist, p.blacklist_radius) or \
                in_blacklist(mx, my, blacklist, p.blacklist_radius):
            sel.n_blacklisted += 1
            continue
        path_m = float(paths[k])
        score = path_m - p.size_gain * min(size_m, p.size_cap)
        sel.candidates.append(Candidate(goal=(gx, gy), goal_cell=(gr, gc), middle=(mx, my),
                                        size_m=size_m, path_m=path_m, score=score,
                                        n_cells=len(cells), cells=cells))
    sel.candidates.sort(key=lambda cnd: cnd.score)
    return sel


def known_area_m2(grid):
    return float(np.count_nonzero(grid.data >= 0)) * grid.resolution ** 2


# ---------------------------------------------------------------- map files

def load_map_yaml(yaml_path, inflate_like_nav2=True, robot_radius=0.15,
                  inflation_radius=0.35, cost_scaling_factor=6.0):
    """Read a map_server .yaml/.pgm pair and turn it into a costmap-like Grid.

    Occupied pixels become 100, free 0, unknown -1. With inflate_like_nav2 the
    free cells near walls get Nav2's inflation values (published 0-100 scale),
    so the offline test sees what frontier_explorer sees on the live costmap.
    """
    import os
    meta = {}
    with open(yaml_path) as f:
        for line in f:
            line = line.split('#', 1)[0].strip()
            if ':' in line:
                k, v = line.split(':', 1)
                meta[k.strip()] = v.strip()
    img_path = meta['image'].strip('"\'')
    if not os.path.isabs(img_path):
        img_path = os.path.join(os.path.dirname(yaml_path), img_path)
    res = float(meta['resolution'])
    origin = [float(v) for v in meta['origin'].strip('[]').split(',')]
    negate = int(meta.get('negate', '0'))
    occ_th = float(meta.get('occupied_thresh', '0.65'))
    free_th = float(meta.get('free_thresh', '0.25'))
    pix = read_pgm(img_path).astype(np.float32)
    maxval = 255.0
    occ = (pix / maxval) if negate else (maxval - pix) / maxval
    data = np.full(pix.shape, UNKNOWN, dtype=np.int16)
    data[occ > occ_th] = LETHAL
    data[occ < free_th] = 0
    data = data[::-1, :].copy()  # image row 0 is the top (highest y)
    grid = Grid(data, res, origin[0], origin[1])
    if inflate_like_nav2:
        grid.data = inflate(grid.data, res, robot_radius, inflation_radius, cost_scaling_factor)
    return grid


def inflate(data, res, robot_radius, inflation_radius, k):
    """Nav2 InflationLayer values on the published 0-100 scale (unknown stays -1)."""
    wall = data >= LETHAL
    d = distance_to_mask(wall, inflation_radius / res + 1.0) * res
    cost = np.where(d <= robot_radius, 253.0,
                    252.0 * np.exp(-k * (d - robot_radius)))
    cost[d > inflation_radius] = 0.0
    pub = np.where(cost >= 253, INSCRIBED, np.round(1 + 97 * (cost - 1) / 251)).astype(np.int16)
    pub[cost <= 0] = 0
    out = data.copy()
    known_free = data == 0
    out[known_free] = pub[known_free]
    out[wall] = LETHAL
    return out


def read_pgm(path):
    """P2 (text) or P5 (binary) PGM to a uint8/uint16 array."""
    with open(path, 'rb') as f:
        raw = f.read()
    magic = raw[:2]
    tokens, pos = [], 2
    while len(tokens) < 3:
        while raw[pos:pos + 1].isspace():
            pos += 1
        if raw[pos:pos + 1] == b'#':
            while raw[pos:pos + 1] not in (b'\n', b''):
                pos += 1
            continue
        start = pos
        while not raw[pos:pos + 1].isspace():
            pos += 1
        tokens.append(int(raw[start:pos]))
    w, h, maxval = tokens
    if magic == b'P5':
        dtype = np.uint8 if maxval < 256 else np.dtype('>u2')
        arr = np.frombuffer(raw[pos + 1:pos + 1 + w * h * np.dtype(dtype).itemsize], dtype=dtype)
    elif magic == b'P2':
        arr = np.array(raw[pos:].split()[:w * h], dtype=np.int32)
    else:
        raise ValueError(f'{path}: not a PGM file ({magic!r})')
    arr = arr.reshape(h, w)
    if maxval != 255:
        arr = (arr.astype(np.float32) * 255.0 / maxval)
    return arr.astype(np.uint8)


def write_pgm_p2(path, img):
    """Write a uint8 array as a plain-text (P2) PGM."""
    h, w = img.shape
    with open(path, 'w') as f:
        f.write(f'P2\n{w} {h}\n255\n')
        for row in img:
            f.write(' '.join(str(int(v)) for v in row) + '\n')
