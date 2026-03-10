
---

# Dog-Legged-Control (ROS 2 Humble)

本项目基于 [qiayuanl/legged_control](https://github.com/qiayuanl/legged_control) 架构，成功移植至 **ROS 2 Humble**。本项目实现了一套完整的四足机器人控制方案。

## 核心技术

* **NMPC (Nonlinear MPC)**：基于 OCS2 库的全身轨迹规划（持续优化中）。
* **WBC (Whole-Body Control)**：支持基于权重分配的多任务全身控制。
* **状态估计**：集成卡尔曼滤波（KF）实现高频机身位姿追踪。

## 快速上手

### 1. 编译

确保处于你的 ROS 2 工作空间根目录：

```bash
colcon build --packages-up-to dog_controllers --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

```

### 2. 启动仿真

```bash
source install/setup.bash
ros2 launch dog_bringup sim.launch.py

```

---


## 后续计划

* 优化 NMPC 模块的移植适配
* 集成机器人运动安全检测与保护机制
* 加入真机部署的定位功能