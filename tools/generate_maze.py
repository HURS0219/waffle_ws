#!/usr/bin/env python3
"""Generate a maze world (SDF), its aligned occupancy map (PGM + YAML) and the
three auto-navigation waypoints for the waffle_ws TurtleBot3 Waffle project.

The maze is a "perfect" maze (unique path between any two cells) built with the
recursive-backtracker algorithm. The robot starts in the bottom-left cell facing
+ x (the wall to its East is forced open so the first corridor is straight and
predictable). Three navigation targets are chosen along the BFS path from the
start to the farthest cell, so the "three-point auto navigation" run is a
meaningful multi-turn route through the maze.

Everything emitted (Gazebo world, Navigation2 map, waypoints) is derived from
this single geometry so they stay perfectly aligned.

Usage:
    python3 tools/generate_maze.py
"""
import argparse
import os
import random
from collections import deque

# ------------------ maze geometry (edit here) ------------------
ROWS = 7           # number of corridor rows (cells)
COLS = 7           # number of corridor columns (cells)
CELL = 1.0         # corridor centre-to-centre distance in metres
WALL_T = 0.15      # wall thickness in metres
WALL_H = 2.0       # wall height in metres
RES = 0.05         # map resolution in m/pixel
MARGIN = 0.30      # map padding beyond the outer walls in metres
SEED = 20240901    # fixed seed -> reproducible maze

FREE = 254         # pgm value for free space
OCCUPIED = 0       # pgm value for occupied space


def build_maze(rows, cols, seed, force_open_east=True):
    """Return (v_walls, h_walls) using recursive backtracker.

    v_walls[r][c] -> vertical wall on the RIGHT of cell (r, c), c in 0..cols
    h_walls[r][c] -> horizontal wall on the TOP of cell (r, c), r in 0..rows
    """
    random.seed(seed)
    v = [[True] * (cols + 1) for _ in range(rows)]
    h = [[True] * cols for _ in range(rows + 1)]

    visited = [[False] * cols for _ in range(rows)]
    stack = [(0, 0)]
    visited[0][0] = True
    dirs = [(0, 1, 0, 1), (0, -1, 0, -1), (1, 0, 1, 0), (-1, 0, -1, 0)]

    while stack:
        r, c = stack[-1]
        nbrs = []
        for dr, dc, _, _ in dirs:
            nr, nc = r + dr, c + dc
            if 0 <= nr < rows and 0 <= nc < cols and not visited[nr][nc]:
                nbrs.append((nr, nc, dr, dc))
        if nbrs:
            nr, nc, dr, dc = random.choice(nbrs)
            if dc == 1:          # neighbour to the right
                v[r][c + 1] = False
            elif dc == -1:       # neighbour to the left
                v[r][c] = False
            elif dr == 1:        # neighbour above
                h[r + 1][c] = False
            elif dr == -1:       # neighbour below
                h[r][c] = False
            visited[nr][nc] = True
            stack.append((nr, nc))
        else:
            stack.pop()

    if force_open_east and COLS > 1:
        # make the first corridor straight towards +x for a deterministic start
        v[0][1] = False
    return v, h


def cell_center(r, c):
    return (c * CELL + CELL / 2.0, r * CELL + CELL / 2.0)


def neighbours(r, c, rows, cols, v, h):
    out = []
    if c + 1 < cols and not v[r][c + 1]:
        out.append((r, c + 1))
    if c - 1 >= 0 and not v[r][c]:
        out.append((r, c - 1))
    if r + 1 < rows and not h[r + 1][c]:
        out.append((r + 1, c))
    if r - 1 >= 0 and not h[r][c]:
        out.append((r - 1, c))
    return out


def bfs_ordering(rows, cols, v, h, start=(0, 0)):
    """Return list of (cell, dist) sorted by BFS distance from start."""
    dist = {start: 0}
    q = deque([start])
    while q:
        r, c = q.popleft()
        for nr, nc in neighbours(r, c, rows, cols, v, h):
            if (nr, nc) not in dist:
                dist[(nr, nc)] = dist[(r, c)] + 1
                q.append((nr, nc))
    return sorted(dist.items(), key=lambda kv: kv[1])


def first_step_toward(v, h, rows, cols, src, dst):
    """Return (dr, dc) of the first grid move from src towards dst."""
    prev = {src: None}
    q = deque([src])
    while q:
        r, c = q.popleft()
        if (r, c) == dst:
            break
        for nr, nc in neighbours(r, c, rows, cols, v, h):
            if (nr, nc) not in prev:
                prev[(nr, nc)] = (r, c)
                q.append((nr, nc))
    cur = dst
    while prev[cur] != src:
        cur = prev[cur]
    return (cur[0] - src[0], cur[1] - src[1])


def pick_targets(rows, cols, v, h, n=3):
    """Pick n targets at ~1/(n), 2/(n)... of the way along start->farthest."""
    ordered = bfs_ordering(rows, cols, v, h)
    farthest, max_dist = ordered[-1]
    if max_dist < n:
        raise RuntimeError('maze too small for %d targets' % n)
    picks = []
    for k in range(1, n + 1):
        want = max_dist * k / float(n)
        cell = min(ordered, key=lambda kv: abs(kv[1] - want))[0]
        if cell not in picks:
            picks.append(cell)
    if farthest not in picks:
        picks[-1] = farthest
    return picks, farthest, max_dist


def wall_segments(v, h, rows, cols):
    """Yield (x, y, sx, sy) for each wall box (centered, horizontal-first axis)."""
    for r in range(rows):
        for c in range(cols + 1):
            if v[r][c]:
                # vertical wall segment between rows r and r+1, at x = c*CELL
                x = c * CELL
                y = (r + 0.5) * CELL
                yield (x, y, WALL_T, CELL)
    for r in range(rows + 1):
        for c in range(cols):
            if h[r][c]:
                # horizontal wall segment between cols c and c+1, at y = r*CELL
                x = (c + 0.5) * CELL
                y = r * CELL
                yield (x, y, CELL, WALL_T)


def is_occupied(x, y, segments):
    for sx, sy, w, d in segments:
        if abs(x - sx) <= w / 2.0 and abs(y - sy) <= d / 2.0:
            return True
    return False


def emit_world(path, v, h, rows, cols):
    segs = list(wall_segments(v, h, rows, cols))
    with open(path, 'w') as f:
        f.write("<?xml version='1.0'?>\n")
        f.write("<sdf version='1.7'>\n")
        f.write("  <world name='maze'>\n")

        f.write("    <light name='sun' type='directional'>\n")
        f.write("      <cast_shadows>0</cast_shadows>\n")
        f.write("      <pose>0 0 10 0 -0 0</pose>\n")
        f.write("      <diffuse>0.8 0.8 0.8 1</diffuse>\n")
        f.write("      <specular>0.2 0.2 0.2 1</specular>\n")
        f.write("      <direction>-0.5 0.1 -0.9</direction>\n")
        f.write("    </light>\n")

        f.write("    <model name='ground_plane'>\n")
        f.write("      <static>1</static>\n")
        f.write("      <link name='link'>\n")
        f.write("        <collision name='collision'>\n")
        f.write("          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>\n")
        f.write("          <surface><friction><ode><mu>100</mu><mu2>50</mu2></ode></friction><contact><ode/></contact><bounce/></surface>\n")
        f.write("        </collision>\n")
        f.write("        <visual name='visual'>\n")
        f.write("          <cast_shadows>0</cast_shadows>\n")
        f.write("          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>\n")
        f.write("          <material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/Grey</name></script></material>\n")
        f.write("        </visual>\n")
        f.write("        <self_collide>0</self_collide>\n")
        f.write("      </link>\n")
        f.write("    </model>\n")

        f.write("    <model name='maze_walls'>\n")
        f.write("      <static>1</static>\n")
        for i, (sx, sy, w, d) in enumerate(segs):
            f.write("      <link name='wall_%d'>\n" % i)
            f.write("        <pose>%s %s %s 0 0 0</pose>\n" % (sx, sy, WALL_H / 2.0))
            f.write("        <collision name='collision'>\n")
            f.write("          <geometry><box><size>%s %s %s</size></box></geometry>\n" % (w, d, WALL_H))
            f.write("        </collision>\n")
            f.write("        <visual name='visual'>\n")
            f.write("          <geometry><box><size>%s %s %s</size></box></geometry>\n" % (w, d, WALL_H))
            f.write("          <material><ambient>0.35 0.55 0.85 1</ambient><diffuse>0.35 0.55 0.85 1</diffuse></material>\n")
            f.write("        </visual>\n")
            f.write("      </link>\n")
        f.write("    </model>\n")

        f.write("    <gravity>0 0 -9.8</gravity>\n")
        f.write("    <physics type='ode'><max_step_size>0.001</max_step_size><real_time_factor>1</real_time_factor><real_time_update_rate>1000</real_time_update_rate></physics>\n")
        f.write("    <scene><ambient>0.5 0.5 0.5 1</ambient><background>0.8 0.8 0.8 1</background><shadows>0</shadows></scene>\n")
        f.write("  </world>\n")
        f.write("</sdf>\n")


def emit_map(pgm_path, yaml_path, v, h, rows, cols):
    segs = list(wall_segments(v, h, rows, cols))
    width = int(round((cols * CELL + 2 * MARGIN) / RES))
    height = int(round((rows * CELL + 2 * MARGIN) / RES))
    origin = (-MARGIN, -MARGIN)

    # PGM row 0 is the TOP of the image, which map_server maps to the HIGHEST y.
    data = bytearray()
    for py in range(height):
        y = origin[1] + (height - 1 - py + 0.5) * RES
        for px in range(width):
            x = origin[0] + (px + 0.5) * RES
            data.append(OCCUPIED if is_occupied(x, y, segs) else FREE)

    with open(pgm_path, 'wb') as f:
        f.write(b"P5\n%d %d\n255\n" % (width, height))
        f.write(data)

    with open(yaml_path, 'w') as f:
        f.write("image: %s\n" % os.path.basename(pgm_path))
        f.write("mode: trinary\n")
        f.write("resolution: %s\n" % RES)
        f.write("origin: [%s, %s, 0.0]\n" % (origin[0], origin[1]))
        f.write("negate: 0\n")
        f.write("occupied_thresh: 0.65\n")
        f.write("free_thresh: 0.25\n")


def emit_waypoints(path, targets, start_cell, rows, cols, v, h):
    """Waypoints in absolute map/world coordinates (map frame == Gazebo world
    frame because slam_toolbox keeps map == odom and the robot is spawned at a
    known world pose)."""
    def yaw_for(i):
        if i < len(targets) - 1:
            dr, dc = first_step_toward(v, h, rows, cols, targets[i], targets[i + 1])
            import math
            return math.atan2(dr, dc)
        return 0.0

    sx, sy = cell_center(*start_cell)
    lines = []
    lines.append("# waypoints for three-point auto navigation (map == world frame)")
    lines.append("# robot spawns at the maze start cell: x=%.2f y=%.2f yaw=0" % (sx, sy))
    lines.append("start: [%s, %s, 0.0]" % (round(sx, 3), round(sy, 3)))
    lines.append("targets:")
    for i, (r, c) in enumerate(targets):
        x, y = cell_center(r, c)
        yaw = yaw_for(i)
        lines.append("  - {x: %s, y: %s, yaw: %s}" % (round(x, 3), round(y, 3), round(yaw, 4)))
    with open(path, 'w') as f:
        f.write("\n".join(lines) + "\n")


def main():
    repo = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    ap = argparse.ArgumentParser()
    ap.add_argument('--world', default=os.path.join(repo, 'src', 'waffle_simulation', 'worlds', 'maze.world'))
    ap.add_argument('--map-dir', default=os.path.join(repo, 'src', 'waffle_navigation', 'maps'))
    ap.add_argument('--config', default=os.path.join(repo, 'src', 'waffle_navigation', 'config', 'waypoints.yaml'))
    args = ap.parse_args()

    os.makedirs(os.path.dirname(args.world), exist_ok=True)
    os.makedirs(args.map_dir, exist_ok=True)
    os.makedirs(os.path.dirname(args.config), exist_ok=True)

    v, h = build_maze(ROWS, COLS, SEED)
    start_cell = (0, 0)
    start = cell_center(*start_cell)
    targets, farthest, max_dist = pick_targets(ROWS, COLS, v, h, n=3)

    emit_world(args.world, v, h, ROWS, COLS)
    emit_map(os.path.join(args.map_dir, 'maze.pgm'),
             os.path.join(args.map_dir, 'maze.yaml'), v, h, ROWS, COLS)
    emit_waypoints(args.config, targets, start_cell, ROWS, COLS, v, h)

    print('maze: %dx%d cells, cell=%.2fm, wall_t=%.2fm' % (ROWS, COLS, CELL, WALL_T))
    print('start world pose (spawn robot here): x=%.2f y=%.2f yaw=0.0'
          % (start[0], start[1]))
    print('farthest cell (%d,%d), BFS dist %d' % (farthest[0], farthest[1], max_dist))
    for i, (r, c) in enumerate(targets):
        print('target %d: cell (%d,%d)  world x=%.2f y=%.2f  map x=%.2f y=%.2f'
              % (i + 1, r, c, cell_center(r, c)[0], cell_center(r, c)[1],
                 (c - start_cell[1]) * CELL, (r - start_cell[0]) * CELL))
    print('wrote:', args.world)
    print('wrote:', os.path.join(args.map_dir, 'maze.pgm'))
    print('wrote:', os.path.join(args.map_dir, 'maze.yaml'))
    print('wrote:', args.config)


if __name__ == '__main__':
    main()
