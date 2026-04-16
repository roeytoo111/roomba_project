# robot_sync_sim

ROS 2 (Humble) package for a **Gazebo Classic** simulation of two differential-drive–style robots: a **leader** that drops trash and a **follower** that navigates to trash and picks it up with a **vacuum gripper** plugin.

## What it does

- **Leader (`robot_a`)** drives on the +X side of the arena, stops at configured drop positions, and spawns trash cubes (via Gazebo `EntityFactory`).
- **Follower (`robot_b`)** starts behind the leader on **negative X**, subscribes to trash poses, drives toward each piece using `cmd_vel`, aligns the vacuum plate over the trash, enables the vacuum, then releases after pickup.
- Motion is implemented with **`libgazebo_ros_planar_move`**: planar velocity from `/robot_a/cmd_vel` and `/robot_b/cmd_vel` (reliable QoS). Odometry from this plugin reports pose in the **world** frame; the follower uses that correctly for goal error.

## Dependencies

- ROS 2 **Humble** (or compatible): `rclpy`, `geometry_msgs`, `std_msgs`, `nav_msgs`, `gazebo_msgs`, `gazebo_ros`, `gazebo_plugins` (vacuum gripper, planar move, joint state publisher).
- **Gazebo Classic** (e.g. Gazebo 11) with `gazebo_ros` bridge.

## Build

From your workspace (e.g. `ros2_ws`):

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select robot_sync_sim
source install/setup.bash
```

## Run

Full simulation (Gazebo, robot spawns, behavior nodes after a short delay):

```bash
ros2 launch robot_sync_sim simulation.launch.py
```

Behavior only (if Gazebo and robots are already running):

```bash
ros2 launch robot_sync_sim behavior.launch.py
```

Useful launch arguments (see `simulation.launch.py`): `use_sim_time`, `start_delay_sec`, `spawn_z`, leader/follower spawn poses, `max_drops` / `max_pickups`, linear/angular speed limits.

Configuration defaults live in `config/sim_params.yaml`.

## Package layout

| Path | Role |
|------|------|
| `launch/` | `simulation.launch.py` (world + spawns + behavior), `behavior.launch.py` (nodes + params) |
| `urdf/` | Xacro for both robots; `common_robot.xacro` shared chassis; planar move + vacuum plugins |
| `worlds/` | `empty.world` with ODE settings |
| `models/trash/` | SDF for trash cube |
| `scripts/` | `leader_node.py`, `follower_node.py` |
| `config/` | `sim_params.yaml` |

## Topics (high level)

- `/robot_a/cmd_vel`, `/robot_b/cmd_vel` — velocity commands.
- Trash announcements: custom string topic from leader; follower parses poses.
- Vacuum: `/robot_b/activate_vacuum` (Bool) and related Gazebo vacuum gripper topics.


