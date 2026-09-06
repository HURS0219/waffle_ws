# TurtleBot3 Waffle 迷宫 · 三点自动导航 (waffle_ws)

ROS 2 Humble + Gazebo Classic 独立工作空间：7×7 **完美迷宫**（走廊净空约 1.35 m，固定 seed、世界与地图同一生成器输出），TurtleBot3 Waffle 在迷宫里做 **slam_toolbox 在线建图 + Navigation2 + 三点自动导航**（已实测自动导航目标 SUCCEEDED）。

## 结构

```
src/
├── waffle_simulation/     Gazebo 迷宫世界 maze.world + Waffle 模型 + 仿真 launch
├── waffle_navigation/     slam.yaml / nav2_params.yaml / waypoints.yaml
│                          launch: slam / navigation / slam_nav(+RViz)
│                          waypoint_follow.py  三点自动导航节点
tools/generate_maze.py     迷宫生成器(世界+地图+航点同一几何来源)
```

三点目标（地图 == Gazebo 世界坐标；机器人出生在起点格 (0.75, 0.75)）：

```yaml
# waffle_navigation/config/waypoints.yaml
targets:
  - {x: 6.75, y: 3.75, yaw: 1.5708}
  - {x: 9.75, y: 5.25, yaw: -1.5708}
  - {x: 6.75, y: 8.25, yaw: 0.0}
```

## 构建

```bash
cd ~/waffle_ws
source /opt/ros/humble/setup.zsh        # bash 用 setup.bash
colcon build --symlink-install
source install/setup.zsh                # bash 用 install/setup.bash
```
zsh 必须 source `.zsh` 版本（source `.bash` 会因 `${BASH_SOURCE[0]}` 为空失败）。

## 运行

终端 A —— 迷宫仿真（默认**不开 RViz**，只有 Gazebo 一个窗口；Waffle 出生在起点格 (0.75,0.75)）：

```bash
source /opt/ros/humble/setup.zsh
source ~/waffle_ws/install/setup.zsh
export LIBGL_ALWAYS_SOFTWARE=1
ros2 launch waffle_simulation waffle.launch.py
```

终端 B —— SLAM + Nav2 + RViz（唯一一个 RViz 窗口）：

```bash
source ~/waffle_ws/install/setup.zsh
export LIBGL_ALWAYS_SOFTWARE=1     # 若 rviz 鼠标悬停消失，去掉这行让 rviz 走 GPU
ros2 launch waffle_navigation slam_nav.launch.py
```

终端 C —— 键盘遥控，**先把三个目标点沿途的迷宫走廊都建进地图**（Nav2 全局代价地图只包含激光实际扫过的墙）：

```bash
source ~/waffle_ws/install/setup.zsh
export TURTLEBOT3_MODEL=waffle
ros2 run turtlebot3_teleop teleop_keyboard
```

终端 D —— 三点自动导航（依次前往上面 3 个目标点）：

```bash
source ~/waffle_ws/install/setup.zsh
ros2 run waffle_navigation waypoint_follow
# 换航点文件：--ros-args -p config_file:=/path/to/waypoints.yaml
```

> Nav2 激活需要 30~45 秒（RViz 里地图/TF 出现、`bt_navigator` active 后才接受目标）。
> 报 `gzserver ... died (exit 255)` 先清残留：`pkill -9 -f gzserver; pkill -9 -f gzclient`。

## 常见问题

- **rviz 鼠标放上去窗口消失/闪没**：WSLg 软件 OpenGL 重绘问题。跑 rviz 的那个终端不要 `export LIBGL_ALWAYS_SOFTWARE=1`，让它走 GPU；仍不行试 `export LIBGL_ALWAYS_INDIRECT=0`。
- **寻路到目标失败**：走廊没建进图（Nav2 全局地图只含扫过的墙），先在 C 终端把路线走一遍；或点一下 RViz 的 “Clear Costmaps”。

## 自定义迷宫 / 航点

改 `tools/generate_maze.py` 顶部（行列、CELL、墙高、SEED 等）后：

```bash
python3 tools/generate_maze.py
colcon build --symlink-install --packages-select waffle_navigation waffle_simulation
```
