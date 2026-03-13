#ifndef ARM_SERIAL_DRIVER_H
#define ARM_SERIAL_DRIVER_H

#include <ros/ros.h>
#include <serial/serial.h>  // 需要安装 ros-$ROS_DISTRO-serial
#include <mutex>
#include <vector>
#include <cstdint>

#include "arm_serial_driver/ArmSerialCommand.h"
#include "arm_serial_driver/ArmSerialFeedback.h"
#include "sensor_msgs/JointState.h"

namespace arm_serial_driver {

class ArmSerialDriver {
public:
    ArmSerialDriver(ros::NodeHandle& nh);
    ~ArmSerialDriver();
    void run();

private:
    void commandCallback(const arm_serial_driver::ArmSerialCommand::ConstPtr& msg);
    bool sendTargetPosition(const boost::array<float, 8>& pos);
    bool readFeedback(boost::array<float, 8>& pos, uint8_t& status);

    // CRC8 计算（多项式 0x07）
    uint8_t crc8(const uint8_t* data, size_t len);

    // 串口配置
    std::string serial_port_;
    int baud_rate_;
    double loop_rate_;

    // 关节名（用于 JointState）
    std::vector<std::string> joint_names_;

    // ROS
    ros::Subscriber cmd_sub_;
    ros::Publisher feedback_pub_;
    ros::Publisher joint_state_pub_;

    // 最新指令（线程安全）
    boost::array<float, 8> latest_command_;
    std::mutex command_mutex_;

    // 串口对象
    serial::Serial serial_;
};

} // namespace arm_serial_driver

#endif