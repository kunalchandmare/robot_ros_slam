#!/usr/bin/env python3
"""test_frontier_selection.py - offline check of frontier_explorer's frontier choice.

No ROS needed: plain Python 3 + NumPy. Run from the repo root:

    python3 src/stereo_eye_launcher/test/test_frontier_selection.py
        builds a small synthetic apartment-like map (see build_synthetic_map)
        and checks the rules below; exit code 0 = all checks passed.
        It also writes the map as maps_test/synthetic.pgm (plain-text P2) +
        .yaml into --out (default /tmp/frontier_test) so you can look at it.

    python3 src/stereo_eye_launcher/test/test_frontier_selection.py \\
            --map src/stereo_eye_launcher/maps/apartment_2026-11-24_10-19-05.yaml \\
            --robot 0.0 0.0
        runs the selection on a map saved by explore_map_saver.py (map frame;
        the robot starts at 0, 0 = the spawn pose) and prints the ranking.
        Nav2's inflation is added like on the live costmap.

Checks on the synthetic map:
  1. the chosen frontier is the one behind the 0.90 m doorway,
  2. nothing behind the 0.25 m wall gap (or the 0.30 m gap) is ever chosen,
     even after the doorway frontier is blacklisted,
  3. no goal lies in the inflation zone: costmap value < approach_max_cost
     and at least min_passage_width / 2 from the nearest wall,
  4. a 0.45 m opening (wider than 0.36 m) IS usable (doors with 5 cm per side),
  5. an unknown blind spot around the robot is not a frontier,
  6. the doorway frontier is reported with its path length, and the
     selection on a full-size 530 x 467 map takes a reasonable time.
"""

import argparse
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'scripts'))

import frontier_core as fc  # noqa: E402

RES = 0.025


def build_synthetic_map():
    """8 m x 5 m building, two rooms, map frame = robot start at (1.5, 2.5).

        y=5 +---------------------------------------+
            |   room A (known)     |  room B         |
            |                      |  (mostly        |
            |   robot *            door 0.90 m       |
            |                      |  unknown)       |
        y=0 +----gap 0.25--gap 0.30-+---gap 0.45----+
                 \\ outside: a little free space seen through the gaps

    - Room A (x 0-4) is fully known free.
    - Room B (x 4-8): only a 1 m strip behind the door is known; beyond is
      unknown -> the "door frontier".
    - South wall of room A has a 0.25 m gap at x=1.0 and a 0.30 m gap at
      x=2.5, and stereo "saw" free space outside behind both (y -0.6..0):
      frontiers that must never be chosen.
    - South wall of room B has a 0.45 m opening at x=6.0 with free space
      outside too: wide enough (> 0.36 m), but only reachable through room B.
    - North wall of room A: free specks outside with no gap at all.
    """
    ox, oy = -0.5, -1.0                               # map covers x -0.5..8.5, y -1..6
    w, h = int(round(9.0 / RES)), int(round(7.0 / RES))
    data = np.full((h, w), fc.UNKNOWN, dtype=np.int16)

    def cells_x(x0, x1):
        return slice(int(round((x0 - ox) / RES)), int(round((x1 - ox) / RES)))

    def cells_y(y0, y1):
        return slice(int(round((y0 - oy) / RES)), int(round((y1 - oy) / RES)))

    t = 0.10  # wall thickness
    data[cells_y(0, 5), cells_x(0, 4)] = 0                       # room A free
    data[cells_y(0, 5), cells_x(4, 5.0)] = 0                     # room B strip behind door
    # walls
    data[cells_y(-t, 0), cells_x(-t, 8 + t)] = fc.LETHAL          # south
    data[cells_y(5, 5 + t), cells_x(-t, 8 + t)] = fc.LETHAL       # north
    data[cells_y(-t, 5 + t), cells_x(-t, 0)] = fc.LETHAL          # west
    data[cells_y(-t, 5 + t), cells_x(8, 8 + t)] = fc.LETHAL       # east
    data[cells_y(-t, 5 + t), cells_x(4, 4 + t)] = fc.LETHAL       # A|B divider
    data[cells_y(2.05, 2.95), cells_x(4, 4 + t)] = 0              # door 0.90 m
    # gaps in the south wall + free space seen outside behind them
    for xc, width in ((1.0, 0.25), (2.5, 0.30)):
        data[cells_y(-t, 0), cells_x(xc - width / 2, xc + width / 2)] = 0
        data[cells_y(-0.6, -t), cells_x(xc - 0.4, xc + 0.4)] = 0
    data[cells_y(-t, 0), cells_x(6.0 - 0.225, 6.0 + 0.225)] = 0   # 0.45 m opening (room B)
    data[cells_y(-0.6, -t), cells_x(5.6, 6.4)] = 0
    data[cells_y(0, 5), cells_x(5.0, 6.6)] = np.where(
        data[cells_y(0, 5), cells_x(5.0, 6.6)] == fc.LETHAL, fc.LETHAL, fc.UNKNOWN)
    # free specks outside the north wall (no gap): seen "through" the wall
    data[cells_y(5 + t, 5.5), cells_x(1.0, 1.6)] = 0
    grid = fc.Grid(data, RES, ox, oy)
    grid.data = fc.inflate(grid.data, RES, 0.15, 0.35, 6.0)
    return grid


def check_goal_clear(grid, cand, params):
    r, c = cand.goal_cell
    v = int(grid.data[r, c])
    wall = grid.data >= fc.LETHAL
    wd = fc.distance_to_mask(wall, 40)[r, c] * grid.resolution
    ok = 0 <= v < params.approach_max_cost and wd >= params.min_passage_width / 2
    return ok, v, wd


def describe(c):
    return (f'goal ({c.goal[0]:.2f}, {c.goal[1]:.2f})  frontier at ({c.middle[0]:.2f}, '
            f'{c.middle[1]:.2f})  size {c.size_m:.2f} m  path {c.path_m:.2f} m  '
            f'score {c.score:.2f}')


def run_synthetic(out_dir):
    grid = build_synthetic_map()
    params = fc.SelectParams()
    robot = (1.5, 2.5)
    failures = []

    def check(cond, text):
        print(('  PASS  ' if cond else '  FAIL  ') + text)
        if not cond:
            failures.append(text)

    os.makedirs(out_dir, exist_ok=True)
    img = np.full(grid.data.shape, 205, dtype=np.uint8)
    img[grid.data >= 0] = 254
    img[grid.data >= fc.LETHAL] = 0
    fc.write_pgm_p2(os.path.join(out_dir, 'synthetic.pgm'), img[::-1, :])
    with open(os.path.join(out_dir, 'synthetic.yaml'), 'w') as f:
        f.write(f'image: synthetic.pgm\nresolution: {RES}\norigin: [{grid.origin_x}, {grid.origin_y}, 0.0]\n'
                'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')
    print(f'Synthetic map written to {out_dir}/synthetic.pgm/.yaml '
          f'({grid.data.shape[1]} x {grid.data.shape[0]} cells)')

    t0 = time.time()
    sel = fc.select_frontiers(grid, robot, params)
    dt = time.time() - t0
    print(f'Selection: {sel.summary()} ({dt:.2f} s)')
    for i, c in enumerate(sel.candidates):
        print(f'  #{i + 1} {describe(c)}')

    def behind_door(c):
        return c.middle[0] > 4.05 and c.middle[1] > 0.0

    def through_small_gap(c):
        return c.middle[1] < 0.0 and c.middle[0] < 4.0

    def outside_north(c):
        return c.middle[1] > 5.0

    check(len(sel.candidates) > 0, 'at least one usable frontier')
    if sel.candidates:
        best = sel.candidates[0]
        check(behind_door(best), f'chosen frontier is behind the 0.90 m door: {describe(best)}')
        check(best.path_m > 2.0, f'path length reported ({best.path_m:.2f} m, robot to door ~2.6 m)')
    check(not any(through_small_gap(c) for c in sel.candidates),
          'no frontier behind the 0.25 m / 0.30 m wall gaps is usable')
    check(not any(outside_north(c) for c in sel.candidates),
          'no frontier behind the solid north wall is usable')
    # blind spot: unknown disk (r 0.2 m) around the robot must not become a goal
    gb = build_synthetic_map()
    rr, cc = np.ogrid[:gb.data.shape[0], :gb.data.shape[1]]
    r0, c0 = gb.to_cell(*robot)
    gb.data[(rr - r0) ** 2 + (cc - c0) ** 2 <= (0.2 / RES) ** 2] = fc.UNKNOWN
    selb = fc.select_frontiers(gb, robot, params)
    check(selb.n_at_robot == 1 and selb.candidates and behind_door(selb.candidates[0]),
          f'unknown blind spot around the robot is ignored ({selb.summary()})')
    all_clear = True
    for c in sel.candidates:
        ok, v, wd = check_goal_clear(grid, c, params)
        all_clear &= ok
        print(f'        goal cost {v} (< {params.approach_max_cost}), '
              f'{wd:.3f} m to the nearest wall (>= {params.min_passage_width / 2:.2f})')
    check(all_clear, 'every goal is outside the inflation zone')
    # check 4: after room B is explored, the 0.45 m opening must be usable.
    grid2 = build_synthetic_map()
    d = grid2.data
    yb = slice(int(round((0.0 - grid2.origin_y) / RES)), int(round((5.0 - grid2.origin_y) / RES)))
    xb = slice(int(round((5.0 - grid2.origin_x) / RES)), int(round((8.0 - grid2.origin_x) / RES)))
    sub = d[yb, xb]
    sub[sub == fc.UNKNOWN] = 0
    grid2.data = fc.inflate(np.where(d >= fc.LETHAL, fc.LETHAL, np.where(d >= 0, 0, -1)).astype(np.int16),
                            RES, 0.15, 0.35, 6.0)
    sel2 = fc.select_frontiers(grid2, robot, params)
    print(f'Room B explored: {sel2.summary()}')
    for i, c in enumerate(sel2.candidates):
        print(f'  #{i + 1} {describe(c)}')
    check(any(5.5 < c.middle[0] < 6.5 and c.middle[1] < 0.0 for c in sel2.candidates),
          'the 0.45 m opening (> 0.36 m) is usable once room B is known')
    check(not any(through_small_gap(c) for c in sel2.candidates),
          'still nothing behind the 0.25 m / 0.30 m gaps')
    # check 2b: blacklist the door frontier -> nothing behind the small gaps either
    bl = [sel.candidates[0].goal] if sel.candidates else []
    sel3 = fc.select_frontiers(grid, robot, params, blacklist=bl)
    print(f'Door frontier blacklisted: {sel3.summary()}')
    check(not any(through_small_gap(c) or outside_north(c) for c in sel3.candidates),
          'with the door blacklisted, no gap/outside frontier is chosen instead')
    check(sel3.n_blacklisted >= 1, 'the blacklisted door frontier is counted as blacklisted')
    # check 5: timing on a full-size map (530 x 467 cells, like Kunal's 2026-11-24 map)
    big = np.full((467, 530), fc.UNKNOWN, dtype=np.int16)
    big[:grid.data.shape[0], :grid.data.shape[1]] = grid.data
    big[250:450, 30:500] = 0
    big[250:450, 260:264] = fc.LETHAL
    bigg = fc.Grid(fc.inflate(big, RES, 0.15, 0.35, 6.0), RES, grid.origin_x, grid.origin_y)
    t0 = time.time()
    sel4 = fc.select_frontiers(bigg, (5.0, 8.0), params)
    dt4 = time.time() - t0
    print(f'Full-size 530 x 467 map: {sel4.summary()} in {dt4:.2f} s')
    check(dt4 < 10.0, f'full-size selection time {dt4:.2f} s (runs in a worker thread)')
    return failures


def run_map(yaml_path, robot, params):
    grid = fc.load_map_yaml(yaml_path)
    print(f'{yaml_path}: {grid.data.shape[1]} x {grid.data.shape[0]} cells, '
          f'{fc.known_area_m2(grid):.1f} m2 known')
    t0 = time.time()
    sel = fc.select_frontiers(grid, robot, params)
    print(f'Selection: {sel.summary()} ({time.time() - t0:.2f} s)')
    if sel.note:
        print('  note: ' + sel.note)
    for i, c in enumerate(sel.candidates):
        print(f'  #{i + 1} {describe(c)}')
    return []


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--map', help='map .yaml saved by map_saver (default: synthetic test)')
    ap.add_argument('--robot', nargs=2, type=float, default=(0.0, 0.0),
                    help='robot x y in the map frame (with --map)')
    ap.add_argument('--out', default='/tmp/frontier_test', help='where the synthetic map goes')
    args = ap.parse_args()
    if args.map:
        failures = run_map(args.map, tuple(args.robot), fc.SelectParams())
    else:
        failures = run_synthetic(args.out)
    if failures:
        print(f'\n{len(failures)} check(s) FAILED')
        sys.exit(1)
    print('\nAll checks passed' if not args.map else '')


if __name__ == '__main__':
    main()
