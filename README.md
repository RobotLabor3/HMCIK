# HMCIK

HMCIK is a ROS package for collision-aware inverse kinematics of 7-DoF humanoid robot arms. See Qinhu Chen et al., *Agriculture* 2026, 16(22), 2556. [A Novel Collision-Free Inverse Kinematics Method for 7-DoF Humanoid Manipulators in Dense Apple Canopies](https://www.mdpi.com/2077-0472/16/22/2556)

## Configuration

[config_double.txt](resource/config_double.txt) contains the settings used for the paper's experiments. For a tighter position tolerance, use [config_double_high_res.txt](resource/config_double_high_res.txt), which sets `eps` to `0.001` mm and allows more iterations. Pass the filename to either arm constructor, for example `LeftArmKine arm("config_double_high_res.txt");`.

| Parameter | Meaning |
| --- | --- |
| `random` | `0` (default) uses random initial configurations for testing. `1` uses a fixed initial configuration. |
| `eps` | Position tolerance in millimeters. The paper's preset uses `1.0` mm. |
| `pc_use` | `1` (default) enables collision checking. `0` disables all collision checks, including self-collision. |
| `scene` | Selects the point cloud. `0` runs without a point cloud. |
| `show` | `1` publishes bounding-box updates during solving on `visualization_marker` for other ROS nodes to subscribe to. Enable ROS publishing with `publish_ros=true` when constructing the arm. The default is `0`. |

## Point-cloud scenes

The files are in [`resource/scenes`](resource/scenes).

| `scene` | Point-cloud file |
| --- | --- |
| `1` | `tree0511SOR.ply` |
| `2` | `04_01.ply` |
| `3` | `07_01.ply` |
| `4` | `10_02.ply` |
| `5` | `zhigan.ply` |
| `6` | `full_tree.ply` |
