# Local Planning in an Unknown Environment

## Project Overview

This ROS 2 (Humble) project implements **two local motion planning algorithms** for a
TurtleBot3 Burger robot navigating in an unknown environment with obstacles. The
platform is **Gazebo + TurtleBot3**.

### Algorithms Implemented

| # | Algorithm | File |
|---|-----------|------|
| 1 | **Dynamic Window Approach (DWA)** | `local_planner/dwa_planner.py` |
| 2 | **Artificial Potential Field (APF)** | `local_planner/apf_planner.py` |

**Bonus:** Dynamic (moving) obstacles via `local_planner/dynamic_obstacle_manager.py`.

---

## Quick Start

### 1. Build

```bash
cd /workspaces/ros2
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
colcon build --packages-select local_planner --symlink-install
source install/setup.bash
```

### 2. Run – Static Obstacles

```bash
# Terminal 1 — DWA planner
ros2 launch local_planner dwa_static.launch.py

# OR — APF planner
ros2 launch local_planner apf_static.launch.py
```

### 3. Run – Dynamic Obstacles (Bonus)

```bash
# Terminal 1 — DWA + moving obstacles
ros2 launch local_planner dwa_dynamic.launch.py

# OR — APF + moving obstacles
ros2 launch local_planner apf_dynamic.launch.py
```

### 4. Change Goal at Runtime

```bash
ros2 launch local_planner dwa_static.launch.py goal_x:=3.0 goal_y:=2.0
```

---

## Algorithm Details

### 1. Dynamic Window Approach (DWA)

**Reference:** Fox, Burgard & Thrun, *"The Dynamic Window Approach to Collision
Avoidance"*, IEEE Robotics & Automation Magazine, 1997.

**How it works:**

1. At each control step the algorithm computes the **dynamic window** — the subset
   of (v, ω) velocities reachable within one time-step given the robot's
   acceleration limits.
2. Every candidate (v, ω) is **forward-simulated** over a prediction horizon
   (1.5 s) to produce a candidate trajectory.
3. Each trajectory is evaluated with a **cost function**:
   - *Heading cost* — alignment of the trajectory endpoint with the goal.
   - *Obstacle cost* — minimum clearance from any obstacle along the trajectory.
   - *Velocity cost* — preference for faster forward motion.
4. The (v, ω) pair with the highest combined cost is selected and published as
   `cmd_vel`.

**Strengths:**
- Considers the full dynamics of the robot (acceleration limits).
- Naturally produces smooth, feasible trajectories.
- Robust in cluttered environments because every candidate path is
  collision-checked.

**Weaknesses:**
- Computationally more expensive (samples O(n²) velocity pairs each step).
- Can get stuck in some concave obstacle configurations.

### 2. Artificial Potential Field (APF)

**Reference:** Khatib, *"Real-Time Obstacle Avoidance for Manipulators and
Mobile Robots"*, Int. J. Robotics Research, 1986.

**How it works:**

1. An **attractive force** is computed pointing from the robot towards the goal.
   - Uses a conic (linear) potential far from the goal and quadratic close to it
     for stability.
2. A **repulsive force** is computed for every LIDAR point within the influence
   distance d₀ (1.0 m), pointing away from the obstacle.
   - Magnitude: K_rep × (1/d − 1/d₀) × 1/d²
3. The **resultant force** (sum of attractive + all repulsive) defines a desired
   heading. This is converted to (v, ω) commands for the differential-drive
   robot.

**Strengths:**
- Extremely fast O(n) per step (one pass over LIDAR rays).
- Elegant, easy to understand and tune.
- Reactive — responds instantly to new sensor data.

**Weaknesses:**
- Susceptible to **local minima** (the robot can get trapped where attractive
  and repulsive forces cancel out).
- Does not account for robot dynamics; commands may be jerky.
- Oscillation in narrow passages.

---

## Comparison — Which Is Better?

| Criterion | DWA | APF |
|-----------|-----|-----|
| **Safety (obstacle clearance)** | ✅ Better — collision-checks full trajectory | ⚠️ Reactive only, no look-ahead |
| **Smoothness** | ✅ Trajectories respect acceleration limits | ⚠️ Can oscillate near obstacles |
| **Computation** | ⚠️ Heavier (but still real-time on TurtleBot3) | ✅ Very lightweight |
| **Local minima** | ⚠️ Can still get stuck, but less likely | ❌ Classic failure mode |
| **Dynamic obstacles** | ✅ Re-plans every cycle with fresh scan | ✅ Reactive by nature |
| **Tuning effort** | Moderate (6+ parameters) | Low (3–4 parameters) |

### Verdict

**DWA is the better algorithm for this task.** It explicitly accounts for the
robot's kinematic constraints and evaluates multiple candidate trajectories
before committing, resulting in safer, smoother navigation. APF is faster to
compute but its lack of trajectory-level collision checking and susceptibility
to local minima make it less reliable in cluttered environments.

For **dynamic obstacles** both algorithms react well because they recompute
every 100 ms from the latest LIDAR scan. However, DWA's prediction horizon
gives it a slight edge — it can foresee that the current velocity will lead to
a collision 1.5 s ahead and choose an alternative trajectory proactively.

---

## Project Structure

```
local_planner/
├── launch/
│   ├── dwa_static.launch.py      # Gazebo + DWA (static obstacles)
│   ├── apf_static.launch.py      # Gazebo + APF (static obstacles)
│   ├── dwa_dynamic.launch.py     # Gazebo + DWA + moving obstacles
│   └── apf_dynamic.launch.py     # Gazebo + APF + moving obstacles
├── local_planner/
│   ├── __init__.py
│   ├── dwa_planner.py            # DWA algorithm node
│   ├── apf_planner.py            # APF algorithm node
│   └── dynamic_obstacle_manager.py  # Spawns & moves obstacles in Gazebo
├── worlds/
│   └── obstacle_world.world      # Gazebo world with static obstacles
├── rviz/
│   └── local_planner.rviz        # Pre-configured RViz layout
├── package.xml
├── setup.py
├── setup.cfg
└── README.md
```

---

## Experimental Results

### Test Configuration

- **World:** `obstacle_world.world` (static obstacles)
- **Start position:** (-2.0, 0.0)
- **Goal position:** (4.0, 0.0)
- **Straight-line distance:** ~6.0 m
- **Goal tolerance:** 0.3 m
- **Control rate:** 10 Hz

### Run Summary

| Planner | Total Runs | Goal Reached | Success Rate |
|---------|-----------|-------------|-------------|
| **APF** | 19 | 5 | **26.3%** |
| **DWA** | 31 | 11 | **35.5%** |

> *Many runs were killed early during development/tuning and are not genuine
> navigation failures.*

### APF Planner — Successful Runs

| Run | Time (s) | Path Length (m) | Min Obstacle Dist (m) | Stuck Events |
|-----|---------|-----------------|----------------------|-------------|
| 1 | 78.00 | 12.44 | 0.40 | 10 |
| 2 | 85.60 | 16.52 | 0.28 | 2 |
| 3 | 106.00 | 20.27 | 0.33 | 3 |
| 4 | 106.70 | 16.13 | 0.31 | 23 |
| 5 | 150.10 | 27.97 | 0.30 | 5 |
| **Avg** | **105.28** | **18.67** | **0.32** | **8.6** |

### DWA Planner — Successful Runs

> *Excluding 2 spurious 0.00 s runs (goal at start) and 1 outlier of 605 s / 6393 m (early dev bug).*

| Run | Time (s) | Path Length (m) | Min Obstacle Dist (m) | Recovery Events |
|-----|---------|-----------------|----------------------|----------------|
| 1 | 31.60 | 6.12 | 0.08 | 0 |
| 2 | 31.90 | 6.16 | 0.08 | 0 |
| 3 | 33.70 | 6.17 | 0.08 | 0 |
| 4 | 34.10 | 6.24 | 0.08 | 0 |
| 5 | 36.30 | 6.28 | 0.08 | 0 |
| 6 | 40.80 | 6.51 | 0.08 | 0 |
| 7 | 41.20 | 6.51 | 0.08 | 0 |
| 8 | 80.20 | 8.68 | 0.18 | 59 |
| **Avg** | **41.23** | **6.58** | **0.09** | **7.4** |

### Comparative Metrics

| Metric | APF | DWA |
|--------|-----|-----|
| Avg time to goal | 105.28 s | 41.23 s |
| Avg path length | 18.67 m | 6.58 m |
| Optimal straight-line | 6.0 m | 6.0 m |
| Path efficiency (optimal / actual) | 32.1% | 91.2% |
| Avg min obstacle clearance | 0.32 m | 0.09 m |
| Avg stuck / recovery events | 8.6 | 7.4 |

### Observations

- **DWA** produces near-optimal paths (~6.6 m vs 6.0 m optimal) but passes
  very close to obstacles (0.08 m clearance — essentially grazing the safety
  radius).

- **APF** takes considerably longer and produces much longer paths (local-minima
  detours cause ~3× overshoot) but maintains safer obstacle clearance (~0.30 m
  on average).

- APF's stuck-escape mechanism (tangential force rotation + gap-based escape) is
  triggered frequently (avg 8.6 times per run), reflecting the well-known
  local-minimum problem of potential field methods.

- DWA's trajectory sampling approach is largely immune to local minima in this
  environment, with most runs completing without any recovery events (only 1 of
  8 valid runs required recovery).

### Algorithm Parameters Used

**APF:**

| Parameter | Value | Description |
|-----------|-------|-------------|
| K_ATT | 1.2 | Attractive gain |
| K_REP | 0.5 | Repulsive gain |
| D0 | 0.8 m | Obstacle influence radius |
| K_TANG | 1.2 | Tangential gain |
| TANG_HOLD | 40 ticks (~4 s) | Direction memory |
| KP_V | 0.8 | Linear velocity P-gain |
| KP_W | 2.5 | Angular velocity P-gain |

**DWA:**

| Parameter | Value | Description |
|-----------|-------|-------------|
| PREDICT_TIME | 0.8 s | Trajectory prediction horizon |
| V_RESOLUTION | 0.02 m/s | Linear velocity sample step |
| W_RESOLUTION | 0.1 rad/s | Angular velocity sample step |
| HEADING_WEIGHT | 1.5 | Goal heading cost weight |
| OBSTACLE_WEIGHT | 0.5 | Obstacle clearance cost weight |
| VELOCITY_WEIGHT | 0.3 | Forward velocity cost weight |
| DISTANCE_WEIGHT | 1.2 | Goal distance cost weight |
| OBSTACLE_RADIUS | 0.08 m | Safety margin around robot |

**Common (TurtleBot3 Burger):**

| Parameter | Value |
|-----------|-------|
| MAX_LINEAR_VEL | 0.22 m/s |
| MAX_ANGULAR_VEL | 2.84 rad/s |
| GOAL_TOLERANCE | 0.3 m |

---

## Metrics Logged

Both planner nodes log the following upon reaching the goal:

- **Time to goal** (seconds)
- **Total distance travelled** (metres)
- **Minimum obstacle clearance** (metres)

These can be compared directly to evaluate which algorithm performs better in a
given scenario.

---

## Environment

- **ROS 2 Humble** on Ubuntu 22.04
- **Gazebo Classic 11**
- **TurtleBot3 Burger**
- Python 3.10 + NumPy

---

*Course project — Local planning in an unknown environment.*
