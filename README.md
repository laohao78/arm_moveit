# THTH0110 机械臂 - ROS 控制系统

## 📋 项目概述

这是一个基于ROS的可编程工业机械臂控制系统，集成了**运动规划（MoveIt）**、**Gazebo仿真**、**串口驱动**和**视觉感知**（RealSense）。系统支持：

- ✅ 仿真环境（Gazebo）与实机控制无缝切换
- ✅ MoveIt运动规划与轨迹执行
- ✅ 多关节组独立控制（机械臂、腕部、夹爪）
- ✅ 实时关节反馈与状态发布
- ✅ RealSense深度相机集成

---

## 🏗️ 项目结构

```
ARM20260120/
├── src/                                  # 源代码
│   ├── arm_serial_driver/               # 串口通信驱动
│   ├── probot_gazebo/                   # Gazebo仿真启动
│   ├── thth0110/                        # URDF机器人模型
│   ├── thth0110_MoveitConfig/           # MoveIt运动规划配置
│   ├── roboticsgroup_gazebo_plugins/    # 从动关节插件
│   └── realsense-ros/                   # RealSense摄像头驱动
├── build/                               # 编译输出
├── devel/                               # 开发环境
└── doc/                                 # 文档资源
```

### 核心模块说明

| 模块 | 功能 | 启动命令 |
|-----|------|--------|
| **probot_gazebo** | 总体启动与场景管理 | `roslaunch probot_gazebo sim_robot.launch` |
| **arm_serial_driver** | 与下位机串口通信 | `roslaunch arm_serial_driver arm_serial_driver.launch` |
| **thth0110_MoveitConfig** | 运动规划与碰撞检测 | 由`probot_gazebo`自动加载 |
| **realsense-ros** | 深度相机与点云 | 集成在仿真或实机启动中 |

---

## 🚀 快速开始

### 1. 环境搭建

**要求：**
- Ubuntu 18.04 / 20.04
- ROS Melodic / Noetic
- Python 3.6+

**初始化工作区：**
```bash
cd ~/Desktop/ARM20260120
source /opt/ros/noetic/setup.bash        # 或 melodic
catkin_make
source devel/setup.bash
```

### 2. 仿真模式

**启动仿真环境 + MoveIt：**
```bash
roslaunch probot_gazebo sim_robot.launch
```

这会启动：
- Gazebo仿真器
- MoveIt规划框架
- RViz可视化
- 所有控制器

**在另一个终端测试运动规划：**
```bash
# 列出可用规划组
rosservice list | grep plan

# 使用MoveIt进行规划与执行
# 在RViz中使用交互式标记进行操作
```

### 3. 实机模式

**启动实机控制：**
```bash
roslaunch probot_gazebo real_robot.launch
```

**查看可用的串口设备：**
```bash
ls /dev/ttyACM* /dev/ttyUSB*
```

**启动串口驱动：**
```bash
roslaunch arm_serial_driver arm_serial_driver.launch
```

---

## 📡 ROS 话题与通信

### 关键话题

#### 串口驱动话题
```
/arm_serial_command          → 发送目标关节位置给下位机
/arm_serial_feedback         → 接收下位机原始反馈数据
/joint_states                → 标准关节状态（ROS标准）
```

#### MoveIt控制话题
```
/thth0110/arm_joint_controller/follow_joint_trajectory/goal
    ↳ 机械臂（j1~j7）轨迹控制

/thth0110/gripper_joint_controller/follow_joint_trajectory/goal
    ↳ 夹爪（j8）控制

/execute_trajectory/goal
    ↳ 高层轨迹执行（MoveIt发送）
```

#### 规划组
- **manipulator_group**：机械臂主体（7个关节）
- **wrist_group**：腕部（独立控制）
- **gripper_group**：夹爪（独立控制）

### 监听话题示例

```bash
# 查看所有活跃话题
rostopic list

# 监听关节命令
rostopic echo /arm_serial_command

# 监听MoveIt目标
rostopic echo /thth0110/arm_joint_controller/follow_joint_trajectory/goal

# 查看关节状态
rostopic echo /joint_states
```

---

## 🔧 串口驱动使用

### 测试下位机通信

```bash
# 编译串口驱动
catkin_make --pkg arm_serial_driver

# 启动驱动节点
roslaunch arm_serial_driver arm_serial_driver.launch

# 在另一终端测试
python src/arm_serial_driver/scripts/test_arm_command.py
```

### 工作流程

| 步骤 | 行为 | 说明 |
|-----|------|------|
| 上电 | 读取当前位置 → 保持不动 | 安全启动 |
| 不运行MoveIt | 持续发送当前位置 | 保持现有状态 |
| 运行MoveIt | 正常跟踪规划轨迹 | 执行运动规划 |
| MoveIt停止 | 保持最后目标位置 | 安全停止 |

---

## 🎮 运动规划与执行

### 架构

```
高层：MoveIt /execute_trajectory
        ↓ (分发)
底层：Controller /follow_joint_trajectory
        ↓ (执行)
硬件：串口驱动 /arm_serial_command
        ↓ (通信)
下位机：机械臂执行
```

### 使用RViz进行规划

1. **启动仿真环境**
   ```bash
   roslaunch probot_gazebo sim_robot.launch
   ```

2. **在RViz中**
   - 选择规划组（manipulator_group/wrist_group/gripper_group）
   - 使用交互式标记调整目标位姿
   - 点击"Plan"进行运动规划
   - 点击"Execute"执行规划轨迹

3. **通过代码规划**
   ```python
   from moveit_commander import MoveGroupCommander
   
   move_group = MoveGroupCommander("manipulator_group")
   move_group.set_named_target("home")
   move_group.go()
   ```

---

## 🎥 视觉集成（RealSense）

### 启用深度相机

相机话题（启用realsense-ros后）：
```
/camera/color/image_raw          → 彩色图像
/camera/depth/image_raw          → 深度图像
/camera/depth/points             → 3D点云
/camera/color/camera_info        → 相机内参
/camera/depth/camera_info        → 深度相机内参
```

### 读取点云

```python
import rospy
from sensor_msgs.msg import PointCloud2
import ros_numpy

def cloud_callback(msg):
    data = ros_numpy.point_cloud2.pointcloud2_to_array(msg)
    # 处理点云数据
    
rospy.Subscriber("/camera/depth/points", PointCloud2, cloud_callback)
rospy.spin()
```

---

## ⚙️ 编译与调试

### 完整编译

```bash
# 编译整个工作区
catkin_make

# 仅编译特定包
catkin_make --pkg arm_serial_driver
catkin_make --pkg thth0110_MoveitConfig
```

### 查看编译错误

```bash
catkin_make --pkg arm_serial_driver -v  # 详细输出
```

### 清理编译

```bash
catkin_make clean
rm -rf build/ devel/
catkin_make
```

---

## 🐛 常见问题

### 1. 串口连接失败
```bash
# 检查设备
ls -la /dev/ttyACM* /dev/ttyUSB*

# 检查权限
sudo chmod 666 /dev/ttyACM0

# 或添加用户到dialout组
sudo usermod -a -G dialout $USER
```

### 2. MoveIt规划失败
- 检查碰撞检测设置
- 验证关节限制配置（limits.yaml）
- 确认初始位置在安全范围内

### 3. Gazebo启动缓慢
```bash
# 设置Gazebo渲染优化
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:$(pwd)/devel/lib

# 或增加启动超时
roslaunch probot_gazebo sim_robot.launch timeout:=60
```

### 4. 清理ROS节点

```bash
# 杀死所有节点
rosnode kill -a

# 清理消息队列
rosgraph_mode clean
```

---

## 📊 ROS计算图示例

```
MoveIt(move_group)
  ├─ Plan: /move_group/goal
  └─ Execute: /execute_trajectory/goal
       ├─ → /thth0110/arm_joint_controller/follow_joint_trajectory/goal
       ├─ → /thth0110/gripper_joint_controller/follow_joint_trajectory/goal
       └─ → Controllers (反馈)
            └─ /joint_states
                 └─ arm_serial_driver (串口)
                      └─ 硬件执行
```

---

## 📦 依赖包

**系统依赖：**
```bash
sudo apt-get install ros-noetic-moveit ros-noetic-gazebo-ros-control \
  ros-noetic-realsense2-camera ros-noetic-joint-state-publisher-gui
```

**核心ROS包：**
- `moveit_core` - 运动规划
- `gazebo_ros_control` - Gazebo控制器
- `realsense2_camera` - RealSense驱动
- `joint_state_publisher` - 关节状态发布

---

## 🔗 相关资源

- [MoveIt官方文档](https://moveit.ros.org/)
- [Gazebo文档](http://gazebosim.org/)
- [ROS官方文档](http://wiki.ros.org/)
- [RealSense ROS](https://github.com/IntelRealSense/realsense-ros)

---

## 📝 许可证

[NULL]

---

## 👤 贡献者

[NULL]

---

## 📞 联系方式

如有问题或建议，请提交Issue或Pull Request。

---

**最后更新**: 2026-04-23
