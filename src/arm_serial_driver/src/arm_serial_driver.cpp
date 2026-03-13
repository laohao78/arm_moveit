#include "arm_serial_driver/arm_serial_driver.h"
#include <sensor_msgs/JointState.h>
#include <ros/console.h>
#include <sstream>
#include <iomanip>

namespace arm_serial_driver {

// CRC8 查表法
static const uint8_t crc8_table[256] = {
    0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
    0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e, 0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
    0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0, 0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
    0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d, 0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,
    0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5, 0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07,
    0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58, 0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
    0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6, 0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24,
    0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b, 0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
    0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f, 0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
    0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92, 0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50,
    0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c, 0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee,
    0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1, 0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
    0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49, 0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
    0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4, 0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16,
    0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a, 0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
    0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7, 0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35,
};

uint8_t ArmSerialDriver::crc8(const uint8_t* data, size_t len) {
    uint8_t crc = 0xff;
    for (size_t i = 0; i < len; ++i) {
        crc = crc8_table[crc ^ data[i]];
    }
    return crc;
}

ArmSerialDriver::ArmSerialDriver(ros::NodeHandle& nh_private) {
    // === 1. 加载参数 ===
    nh_private.param<std::string>("serial_port", serial_port_, "/dev/ttyACM0");
    nh_private.param<int>("baud_rate", baud_rate_, 921600);
    nh_private.param<double>("loop_rate", loop_rate_, 50.0);
    nh_private.param<std::vector<std::string>>("joint_names", joint_names_,
        {"j1", "j2", "j3", "j4", "j5", "j6", "j7", "j8"});

    if (joint_names_.size() != 8) {
        ROS_ERROR("joint_names must have exactly 8 elements!");
        ros::shutdown();
    }

    latest_command_.fill(0.0f);

    // === 2. 打开串口 ===
    try {
        serial_.setPort(serial_port_);
        serial_.setBaudrate(baud_rate_);
        serial::Timeout timeout = serial::Timeout::simpleTimeout(100); // 延长超时
        serial_.setTimeout(timeout);
        serial_.open();
        ROS_INFO("Serial port %s opened at %d baud", serial_port_.c_str(), baud_rate_);
    } catch (const std::exception& e) {
        ROS_ERROR("Failed to open serial port: %s", e.what());
        ros::shutdown();
    }

    // === 3. 尝试读取初始位置（利用绝对编码器）===
    boost::array<float, 8> initial_pos;
    uint8_t status;
    bool got_initial = false;

    ROS_INFO("Attempting to read initial joint positions from hardware...");

    for (int attempt = 0; attempt < 20 && ros::ok(); ++attempt) {
        ros::Duration(0.1).sleep();

        // 清空串口缓冲（避免旧数据干扰）
        while (serial_.available() > 0) {
            uint8_t dummy;
            serial_.read(&dummy, 1);
        }

        if (readFeedback(initial_pos, status)) {
            got_initial = true;
            break;
        }
    }

    if (got_initial) {
        {
            std::lock_guard<std::mutex> lock(command_mutex_);
            latest_command_ = initial_pos;
        }
        ROS_INFO("✅ Initial joint positions acquired. Robot will hold current pose.");
    } else {
        ROS_WARN("❌ Failed to read initial positions. Defaulting to zeros.");
        // 保留 latest_command_ 为全零（安全兜底）
    }

    // === 4. 创建全局 NodeHandle 并设置通信 ===
    ros::NodeHandle nh;
    cmd_sub_ = nh.subscribe("arm_serial_command", 10, &ArmSerialDriver::commandCallback, this);
    feedback_pub_ = nh.advertise<arm_serial_driver::ArmSerialFeedback>("arm_serial_feedback", 10);
    joint_state_pub_ = nh.advertise<sensor_msgs::JointState>("joint_states", 10);
}

ArmSerialDriver::~ArmSerialDriver() {
    if (serial_.isOpen()) serial_.close();
}

void ArmSerialDriver::commandCallback(const arm_serial_driver::ArmSerialCommand::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(command_mutex_);
    latest_command_ = msg->position;
}

bool ArmSerialDriver::sendTargetPosition(const boost::array<float, 8>& pos) {
    if (pos.size() != 8) return false;

    std::vector<uint8_t> frame(35);
    frame[0] = 0xAA;
    frame[1] = 0x55;

    uint8_t* ptr = frame.data() + 2;
    for (int i = 0; i < 8; ++i) {
        float f = pos[i];
        uint8_t* f_bytes = reinterpret_cast<uint8_t*>(&f);
        *ptr++ = f_bytes[0];
        *ptr++ = f_bytes[1];
        *ptr++ = f_bytes[2];
        *ptr++ = f_bytes[3];
    }
    frame[34] = crc8(frame.data(), 34);

    try {
        serial_.write(frame);

        std::stringstream ss;
        ss << "\033[32m[TX] ";
        for (size_t i = 0; i < joint_names_.size(); ++i) {
            ss << joint_names_[i] << ": " << std::fixed << std::setprecision(3) << pos[i];
            if (i < joint_names_.size() - 1) ss << ", ";
        }
        ss << "\033[0m";
        ROS_INFO_STREAM(ss.str());

        return true;
    } catch (...) {
        ROS_ERROR_THROTTLE(1.0, "Serial write failed");
        return false;
    }
}

bool ArmSerialDriver::readFeedback(boost::array<float, 8>& pos, uint8_t& status) {
    static std::vector<uint8_t> buffer;
    const size_t FRAME_SIZE = 36;

    try {
        while (serial_.available() > 0) {
            uint8_t byte;
            serial_.read(&byte, 1);
            buffer.push_back(byte);

            if (buffer.size() >= 2) {
                if (buffer[0] != 0x55 || buffer[1] != 0xAA) {
                    buffer.erase(buffer.begin());
                    continue;
                }
            }

            if (buffer.size() == FRAME_SIZE) {
                uint8_t computed_crc = crc8(buffer.data(), FRAME_SIZE - 1);
                if (computed_crc == buffer[FRAME_SIZE - 1]) {
                    const uint8_t* ptr = buffer.data() + 2;
                    for (int i = 0; i < 8; ++i) {
                        float f;
                        memcpy(&f, ptr, sizeof(float));
                        pos[i] = f;
                        ptr += sizeof(float);
                    }
                    status = *ptr;

                    std::stringstream ss;
                    ss << "\033[33m[RX] ";
                    for (size_t i = 0; i < joint_names_.size(); ++i) {
                        ss << joint_names_[i] << ": " << std::fixed << std::setprecision(3) << pos[i];
                        if (i < joint_names_.size() - 1) ss << ", ";
                    }
                    ss << " | status: " << (int)status << "\033[0m";
                    ROS_INFO_STREAM(ss.str());

                    buffer.clear();
                    return true;
                } else {
                    ROS_WARN_THROTTLE(1.0, "CRC error in feedback frame");
                    buffer.clear();
                    return false;
                }
            }

            if (buffer.size() > FRAME_SIZE) {
                buffer.clear();
            }
        }
    } catch (...) {
        ROS_ERROR_THROTTLE(1.0, "Serial read error");
        buffer.clear();
    }
    return false;
}

void ArmSerialDriver::run() {
    ros::Rate rate(loop_rate_);
    boost::array<float, 8> current_pos;
    current_pos.assign(0.0f);
    uint8_t status = 0;

    while (ros::ok()) {
        boost::array<float, 8> target;
        {
            std::lock_guard<std::mutex> lock(command_mutex_);
            target = latest_command_;
        }

        sendTargetPosition(target);

        if (readFeedback(current_pos, status)) {
            arm_serial_driver::ArmSerialFeedback feedback_msg;
            feedback_msg.position = current_pos;
            feedback_msg.status = status;
            feedback_pub_.publish(feedback_msg);

            sensor_msgs::JointState js_msg;
            js_msg.header.stamp = ros::Time::now();
            js_msg.name = joint_names_;
            js_msg.position.resize(8);
            for (size_t i = 0; i < 8; ++i) {
                js_msg.position[i] = static_cast<double>(current_pos[i]);
            }
            joint_state_pub_.publish(js_msg);
        }

        ros::spinOnce();
        rate.sleep();
    }
}

} // namespace arm_serial_driver