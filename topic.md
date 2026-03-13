
## ROS话题含义分析

### 机器人与仿真相关
- `/clock`：仿真时间同步，Gazebo发布。
- `/gazebo/link_states`：Gazebo中所有链接的状态（位置、姿态等）。
- `/gazebo/model_states`：Gazebo中所有模型的状态。
- /gazebo/parameter_descriptions：Gazebo参数描述。
- /gazebo/parameter_updates：Gazebo参数更新。
- /gazebo/performance_metrics：Gazebo性能指标。
- `/gazebo/set_link_state`：设置Gazebo中链接状态。
- `/gazebo/set_model_state`：设置Gazebo中模型状态。

### 机器人关节与控制
- `/joint_states`：全局关节状态（位置、速度、努力），由joint_state_publisher发布。
- `/thth0110/joint_states`：thth0110机器人关节状态。
- `/thth0110/arm_joint_controller/command`：机械臂关节控制命令。
- `/thth0110/arm_joint_controller/state`：机械臂关节控制器状态反馈。
- `/thth0110/arm_joint_controller/follow_joint_trajectory/goal`：机械臂轨迹控制目标。
- `/thth0110/arm_joint_controller/follow_joint_trajectory/result`：机械臂轨迹控制结果。
- /thth0110/arm_joint_controller/follow_joint_trajectory/cancel：机械臂轨迹控制取消。
- /thth0110/arm_joint_controller/follow_joint_trajectory/feedback：机械臂轨迹控制反馈。
- /thth0110/arm_joint_controller/follow_joint_trajectory/status：机械臂轨迹控制状态。
- `/thth0110/gripper_joint_controller/command`：夹爪控制命令。
- `/thth0110/gripper_joint_controller/state`：夹爪控制器状态反馈。
- `/thth0110/gripper_joint_controller/follow_joint_trajectory/goal`：夹爪轨迹控制目标。
- `/thth0110/gripper_joint_controller/follow_joint_trajectory/result`：夹爪轨迹控制结果。
- /thth0110/gripper_joint_controller/follow_joint_trajectory/cancel：夹爪轨迹控制取消。
- /thth0110/gripper_joint_controller/follow_joint_trajectory/feedback：夹爪轨迹控制反馈。
- /thth0110/gripper_joint_controller/follow_joint_trajectory/status：夹爪轨迹控制状态。

### MoveIt! 运动规划与执行
- `/move_group/goal`：MoveIt!运动规划目标。
- `/move_group/result`：MoveIt!运动规划结果。
- /move_group/cancel：MoveIt!运动规划取消。
- /move_group/feedback：MoveIt!运动规划反馈。
- /move_group/status：MoveIt!运动规划状态。
- `/planning_scene`：当前规划场景信息。
- `/planning_scene_world`：世界场景信息。
- /pickup/goal：抓取动作目标。
- /pickup/result：抓取动作结果。
- /pickup/cancel：抓取动作取消。
- /pickup/feedback：抓取动作反馈。
- /pickup/status：抓取动作状态。
- /place/goal：放置动作目标。
- /place/result：放置动作结果。
- /place/cancel：放置动作取消。
- /place/feedback：放置动作反馈。
- /place/status：放置动作状态。
- /sequence_move_group/goal：序列动作目标。
- /sequence_move_group/result：序列动作结果。
- /sequence_move_group/cancel：序列动作取消。
- /sequence_move_group/feedback：序列动作反馈。
- /sequence_move_group/status：序列动作状态。
- `/execute_trajectory/goal`：执行轨迹目标。
- `/execute_trajectory/result`：执行轨迹结果。
- /execute_trajectory/cancel：执行轨迹取消。
- /execute_trajectory/feedback：执行轨迹反馈。
- /execute_trajectory/status：执行轨迹状态。
- `/trajectory_execution_event`：轨迹执行事件。

### TF坐标变换
- `/tf`：实时坐标变换。
- `/tf_static`：静态坐标变换。

### 视觉与传感器
- `/camera/color/camera_info`：彩色相机内参。
- `/camera/color/image_raw`：彩色相机原始图像。
- /camera/color/image_raw/compressed：压缩彩色图像。
- /camera/color/image_raw/compressed/parameter_descriptions：压缩参数描述。
- /camera/color/image_raw/compressed/parameter_updates：压缩参数更新。
- /camera/color/image_raw/compressedDepth：压缩深度图像。
- /camera/color/image_raw/compressedDepth/parameter_descriptions：深度压缩参数描述。
- /camera/color/image_raw/compressedDepth/parameter_updates：深度压缩参数更新。
- /camera/color/image_raw/theora：theora编码图像。
- /camera/color/image_raw/theora/parameter_descriptions：theora参数描述。
- /camera/color/image_raw/theora/parameter_updates：theora参数更新。
- `/camera/depth/camera_info`：深度相机内参。
- `/camera/depth/image_raw`：深度相机原始图像。
- `/camera/depth/points`：深度点云。
- /d455/parameter_descriptions：D455相机参数描述。
- /d455/parameter_updates：D455相机参数更新。

### 物体识别与交互
- `/attached_collision_object`：附着碰撞物体信息。
- `/collision_object`：碰撞物体信息。
- `/recognized_object_array`：识别到的物体数组。

### RViz与可视化
- /rviz_gugugaga_503_7801222571864440105/motionplanning_planning_scene_monitor/parameter_descriptions：RViz规划场景监控参数描述。
- /rviz_gugugaga_503_7801222571864440105/motionplanning_planning_scene_monitor/parameter_updates：RViz规划场景监控参数更新。
- /rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/feedback：RViz交互式标记反馈。
- `/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/update`：RViz交互式标记更新。
- `/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/update_full`：RViz交互式标记全量更新。

### ROS系统与日志
- `/rosout`：ROS日志输出。
- `/rosout_agg`：聚合日志输出。


