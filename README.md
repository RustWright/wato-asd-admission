# WATonomous ASD Admissions Assignment

## Prerequisite Installation
These steps are to setup the monorepo to work on your own PC. We utilize docker to enable ease of reproducibility and deployability.

> Why docker? It's so that you don't need to download any coding libraries on your bare metal pc, saving headache :3

1. This assignment is supported on Linux Ubuntu >= 22.04, Windows (WSL), and MacOS. This is standard practice that roboticists can't get around. To setup, you can either setup an [Ubuntu Virtual Machine](https://ubuntu.com/tutorials/how-to-run-ubuntu-desktop-on-a-virtual-machine-using-virtualbox#1-overview), setting up [WSL](https://learn.microsoft.com/en-us/windows/wsl/install), or setting up your computer to [dual boot](https://opensource.com/article/18/5/dual-boot-linux). You can find online resources for all three approaches.
2. Once inside Linux, [Download Docker Engine using the `apt` repository](https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository)
3. You're all set! You can begin the assignment by visiting the WATonomous Wiki.

Link to Onboarding Assignment: https://wiki.watonomous.ca/

---

# AI Assistance

I used Claude Code throughout this assignment.

**I wrote myself:** all four algorithm cores, meaning the costmap scan-to-grid conversion and
inflation, the map_memory fusion transform, the A\* search, and the Pure Pursuit controller. I also
wrote the ROS node files for costmap and map_memory, and did all the parameter tuning.

**Claude wrote:** the ROS node wiring for the planner and control nodes, the `CMakeLists.txt` and
`package.xml` dependency changes, the parameter files, and the verification runs. It also reviewed my
code and explained C++ and ROS constructs that were new to me, since I came to this from Rust.

**Why the split moved:** I wrote both halves of the costmap and map_memory nodes myself, so by the
time I reached the planner and control nodes I had already written that kind of ROS wiring twice. With
the deadline close, I handed those two node files over deliberately, to spend the remaining time on
the algorithms rather than on subscriptions and message conversion I had already practised.

`AI_ASSISTANCE.md` is a full per-item log of what each of us did, written as the work happened rather
than reconstructed afterwards. It records the places Claude was wrong and I corrected it, as well as
the reverse.

---

# Implementation Notes

Design decisions and their tradeoffs, for the four nodes in `src/robot/`.

## Costmap (`/lidar` → `/costmap`)

400 × 400 @ 0.1 m, robot-centred. Cells initialise to **0 (free), not −1 (unknown)**: simpler, renders
cleanly, and gives the planner a traversable field. The tradeoff is real. The costmap asserts "clear"
for cells the lidar never reached, rather than distinguishing unknown from free.

Inflation runs inside the obstacle-marking loop, iterating obstacles rather than sweeping all 160k
cells.

## Map memory (`/costmap` + `/odom/filtered` → `/map`)

200 × 200 @ 0.2 m, world-fixed, origin `-width * resolution / 2`. A fixed world-sized grid only works
because the arena is known-bounded; a real system grows the map or rolls a window as the robot drives.

**The costmap is deliberately the finer grid** (0.1 m against the map's 0.2 m). A coarser source would
leave holes when its cells are scattered into the map; at 2:1, four local cells land in every covered
global cell. Fusion is a *forward* map, meaning it walks the local cells, transforms each one into the
world frame and scatters it into the global grid. That direction needs no inverse transform, and at
2:1 it cannot leave gaps.

Merge is `max`, so one obstacle cell in the costmap marks a full 0.2 m map cell. The map is
deliberately fatter than the costmap, which is the conservative direction for a planner.

**Localization is simulator ground truth, not odometry.** `odometry_spoof` republishes the TF
`sim_world → robot/chassis/lidar`, so the pose has no drift and the map needs no SLAM, no loop closure
and no pose-graph correction. Real odometry would make this a different problem.

## Planner (`/map` + `/goal_point` + `/odom/filtered` → `/path`)

A\* over the fused map, 8-connected, with a Euclidean heuristic.

`std::priority_queue` has **no decrease-key**, so the open set uses **lazy deletion**: a cell is pushed
again whenever its `g` improves, and stale copies are discarded on pop by checking the closed set.
Correct because the first time a cell reaches the top it carries the lowest `f` any copy will have.
"Already discovered" is answered by the `g` map, not by the queue.

`g` accumulates step cost **plus `cost_weight × cell cost`**, so the inflation gradient makes the
planner prefer the roomy route over shaving a corner.

## Control (`/path` + `/odom/filtered` → `/cmd_vel`)

Pure Pursuit at constant `linear_speed`. The lookahead target is transformed into the robot frame with
a dot/cross product against the robot's forward axis, and the arc curvature follows from the target's
lateral offset `y_r` and chord length `L` as `2·y_r / L²`. No angle is ever formed, so there is no
angle-wrapping to get wrong. `max_angular_speed` clamps the singularity as `L → 0`.

## Tuning

| parameter | value | why |
|---|---|---|
| `inflation_radius` | 1.0 | Wider clearance; visible as ~1.1 m berth around obstacles. |
| `linear_speed` | 0.8 | |
| `lookahead_distance` | 1.4 | Coupled to speed. The ratio of lookahead to speed is what sets Pure Pursuit stability. |
| `max_angular_speed` | 2.0 | Coupled to speed. The tightest achievable turn is `v / max_angular_speed`. |
| `cost_weight` | 0.15 | Raised until paths cleared obstacles by more than the robot's half-width, since Pure Pursuit cuts corners. |
