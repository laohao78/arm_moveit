|Movelt!机器人控制框架|
|:--:|
|![alt text](/doc/image.png)|


## [***<ins>probot_gazebo***](src/probot_gazebo)：总启动
```sh
roslaunch probot_gazebo sim_robot.launch
roslaunch probot_gazebo real_robot.launch
```
## [***<ins>thth0110***](src/thth0110)：SW转urdf的文件

## [***<ins>thth0110_MoveitConfig***](src/thth0110_MoveitConfig)：Moveit配置

## [***<ins>roboticsgroup_gazebo_plugins***](src/roboticsgroup_gazebo_plugins)：提供从动关节插件

## [***<ins>arm_serial_driver***](src/arm_serial_driver)：串口
```sh
# 查看端口
ls /dev/ttyACM* /dev/ttyUSB* /dev/ttyShaobing 
# 单独编译
catkin_make --pkg arm_serial_driver
# 启动
roslaunch arm_serial_driver arm_serial_driver.launch
# 测试下位机
python src/arm_serial_driver/scripts/test_arm_command.py
# 节点
rostopic list
    /arm_serial_command
    /arm_serial_feedback
    /joint_states
    /rosout
    /rosout_agg
    /thth0110/arm_joint_controller/follow_joint_trajectory/goal
    /thth0110/gripper_joint_controller/follow_joint_trajectory/goal
```

## [***<ins>realsense-ros***](src/realsense-ros)：realsense Gazebo 仿真
