# robot_sync_sim

ROS 2 (Humble) + Gazebo Classic: leader drops trash cubes; follower uses `gazebo_ros_vacuum_gripper` to attract them.

## Vacuum pickup (important)

The stock `gazebo_ros_vacuum_gripper` applies a **small attraction force** toward `vacuum_link` when the gripper is on and a link is within `max_distance` — it does **not** weld the object. Driving forward right after enabling the vacuum tends to **push** the cube with the chassis. The follower therefore **stops** (`cmd_vel` zero), turns the vacuum **on**, waits for `/robot_b/grasping` to become true, holds briefly, then turns the vacuum **off**. Trash collision uses **reduced friction** so the cube can slide into the suction zone instead of only sliding from wheel pushes.

Tune: `grasp_timeout_s`, `post_grasp_hold_s`, `post_mission_vacuum_duration_s` (vacuum stays on after the last pickup), `goal_tolerance_m`, `vacuum_forward_offset_m`, and URDF `max_distance`.

## Build

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select robot_sync_sim
source install/setup.bash
```

## Run

```bash
ros2 launch robot_sync_sim simulation.launch.py
```

## License

Apache-2.0
