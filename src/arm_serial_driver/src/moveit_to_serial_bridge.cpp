#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>               // ← 新增
#include <arm_serial_driver/ArmSerialCommand.h>
#include <boost/array.hpp>
#include <mutex>

class MoveItToSerialBridge {
public:
    MoveItToSerialBridge()
        : nh_(),
          as_(nh_, "/thth0110/arm_joint_controller/follow_joint_trajectory",
              boost::bind(&MoveItToSerialBridge::executeArmCB, this, _1), false),
          gripper_as_(nh_, "/thth0110/gripper_joint_controller/follow_joint_trajectory",
              boost::bind(&MoveItToSerialBridge::executeGripperCB, this, _1), false)
    {
        pub_ = nh_.advertise<arm_serial_driver::ArmSerialCommand>("arm_serial_command", 10);
        joint_state_sub_ = nh_.subscribe("joint_states", 10, &MoveItToSerialBridge::jointStateCallback, this); // ← 订阅反馈
        timer_ = nh_.createTimer(ros::Duration(1.0 / 50.0), &MoveItToSerialBridge::timerCallback, this);

        // 初始设为0，但会尽快被 /joint_states 覆盖
        command_.fill(0.0f);
        initialized_from_hardware_ = false;

        as_.start();
        gripper_as_.start();

        ROS_INFO("MoveItToSerialBridge: Action servers started. Waiting for /joint_states to initialize...");
    }

private:
    // === 从硬件反馈初始化 command_ ===
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    if (initialized_from_hardware_) return; // 只初始化一次

    std::lock_guard<std::mutex> lock(mutex_);

    bool updated_any = false;

    for (size_t i = 0; i < msg->name.size(); ++i) {
        if (i >= msg->position.size()) continue; // 安全检查
        const std::string& name = msg->name[i];
        float pos = static_cast<float>(msg->position[i]);

        if (name == "j1") { command_[0] = pos; updated_any = true; }
        else if (name == "j2") { command_[1] = pos; updated_any = true; }
        else if (name == "j3") { command_[2] = pos; updated_any = true; }
        else if (name == "j4") { command_[3] = pos; updated_any = true; }
        else if (name == "j5") { command_[4] = pos; updated_any = true; }
        else if (name == "j6") { command_[5] = pos; updated_any = true; }
        else if (name == "j7") { command_[6] = pos; updated_any = true; }
        else if (name == "j8") { command_[7] = pos; updated_any = true; }
        // 忽略其他关节名
    }

    if (updated_any) {
        initialized_from_hardware_ = true;
        ROS_INFO("✅ Initialized command_ from /joint_states:");
        ROS_INFO("    j1=%.3f, j2=%.3f, j3=%.3f, j4=%.3f, j5=%.3f, j6=%.3f, j7=%.3f, j8=%.3f",
                 command_[0], command_[1], command_[2], command_[3],
                 command_[4], command_[5], command_[6], command_[7]);
    }
}

    // === Arm Action Callback (j1～j7) ===
    void executeArmCB(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal) {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!initialized_from_hardware_) {
            ROS_ERROR("❌ Rejecting ARM trajectory: not yet initialized from /joint_states!");
            control_msgs::FollowJointTrajectoryResult result;
            as_.setAborted(result, "Not initialized");
            return;
        }

        if (goal->trajectory.points.empty()) {
            control_msgs::FollowJointTrajectoryResult result;
            as_.setSucceeded(result);
            return;
        }

        current_arm_trajectory_ = goal->trajectory;
        trajectory_start_time_ = ros::Time::now();
        executing_ = true;
        arm_done_ = false;

        ROS_INFO("Received new ARM trajectory with %zu points", current_arm_trajectory_.points.size());
    }

    // === Gripper Action Callback (j8) ===
    void executeGripperCB(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal) {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!initialized_from_hardware_) {
            ROS_ERROR("❌ Rejecting GRIPPER trajectory: not yet initialized from /joint_states!");
            control_msgs::FollowJointTrajectoryResult result;
            gripper_as_.setAborted(result, "Not initialized");
            return;
        }


        if (goal->trajectory.points.empty()) {
            control_msgs::FollowJointTrajectoryResult result;
            gripper_as_.setSucceeded(result);
            return;
        }

        current_gripper_trajectory_ = goal->trajectory;
        gripper_start_time_ = ros::Time::now();
        gripper_executing_ = true;
        gripper_done_ = false;

        ROS_INFO("Received new GRIPPER trajectory with %zu points", current_gripper_trajectory_.points.size());
    }

    // === 定时器回调：局部更新 + 持续发布 ===
    void timerCallback(const ros::TimerEvent&) {
        std::lock_guard<std::mutex> lock(mutex_);

        // 🟢 关键：不再清零！command_ 是持久状态

        // ========== 处理 Arm 轨迹 (j1～j7) ==========
        if (executing_ && !current_arm_trajectory_.points.empty()) {
            double elapsed = (ros::Time::now() - trajectory_start_time_).toSec();
            const auto& traj = current_arm_trajectory_;

            size_t idx = 0;
            for (size_t i = 0; i < traj.points.size(); ++i) {
                if (traj.points[i].time_from_start.toSec() <= elapsed) {
                    idx = i;
                } else {
                    break;
                }
            }

            const auto& point = traj.points[idx];
            const auto& names = traj.joint_names;
            for (size_t i = 0; i < names.size() && i < point.positions.size(); ++i) {
                float val = static_cast<float>(point.positions[i]);
                const std::string& name = names[i];
                if (name == "j1") command_[0] = val;
                else if (name == "j2") command_[1] = val;
                else if (name == "j3") command_[2] = val;
                else if (name == "j4") command_[3] = val;
                else if (name == "j5") command_[4] = val;
                else if (name == "j6") command_[5] = val;
                else if (name == "j7") command_[6] = val;
            }

            if (idx >= traj.points.size() - 1) {
                arm_done_ = true;
            }
        }

        // ========== 处理 Gripper 轨迹 (j8) ==========
        if (gripper_executing_ && !current_gripper_trajectory_.points.empty()) {
            double elapsed = (ros::Time::now() - gripper_start_time_).toSec();
            const auto& traj = current_gripper_trajectory_;

            size_t idx = 0;
            for (size_t i = 0; i < traj.points.size(); ++i) {
                if (traj.points[i].time_from_start.toSec() <= elapsed) {
                    idx = i;
                } else {
                    break;
                }
            }

            if (!traj.points[idx].positions.empty()) {
                command_[7] = static_cast<float>(traj.points[idx].positions[0]);
            }

            if (idx >= traj.points.size() - 1) {
                gripper_done_ = true;
            }
        }

        // ========== 总是发布当前完整命令（关键修复！）==========
        arm_serial_driver::ArmSerialCommand out_msg;
        out_msg.position = command_;
        pub_.publish(out_msg);

        // ========== 抢占处理 ==========
        if (as_.isPreemptRequested() && executing_) {
            ROS_WARN("Arm trajectory preempted!");
            executing_ = false;
            arm_done_ = true;
            control_msgs::FollowJointTrajectoryResult result;
            as_.setPreempted(result);
        }

        if (gripper_as_.isPreemptRequested() && gripper_executing_) {
            ROS_WARN("Gripper trajectory preempted!");
            gripper_executing_ = false;
            gripper_done_ = true;
            control_msgs::FollowJointTrajectoryResult result;
            gripper_as_.setPreempted(result);
        }

        // ========== 成功处理 ==========
        if (arm_done_ && executing_) {
            executing_ = false;
            control_msgs::FollowJointTrajectoryResult result;
            as_.setSucceeded(result);
            ROS_INFO("Arm trajectory succeeded.");
        }

        if (gripper_done_ && gripper_executing_) {
            gripper_executing_ = false;
            control_msgs::FollowJointTrajectoryResult result;
            gripper_as_.setSucceeded(result);
            ROS_INFO("Gripper trajectory succeeded.");
        }

        ROS_DEBUG_THROTTLE(2.0,
            "Sent: j1=%.3f, j2=%.3f, j3=%.3f, j4=%.3f, j5=%.3f, j6=%.3f, j7=%.3f, j8=%.3f",
            command_[0], command_[1], command_[2], command_[3],
            command_[4], command_[5], command_[6], command_[7]);
    }

    // ====== ROS Handles ======
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Subscriber joint_state_sub_;   // ← 新增
    ros::Timer timer_;

    // ====== Action Servers ======
    actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> as_;
    actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> gripper_as_;

    // ====== Trajectory Data ======
    trajectory_msgs::JointTrajectory current_arm_trajectory_;
    trajectory_msgs::JointTrajectory current_gripper_trajectory_;

    // ====== Command State ======
    boost::array<float, 8> command_;           // 持久化命令
    bool initialized_from_hardware_ = false;   // 是否已从硬件初始化

    // ====== Execution Flags ======
    bool executing_ = false;
    bool gripper_executing_ = false;
    bool arm_done_ = false;
    bool gripper_done_ = false;
    ros::Time trajectory_start_time_;
    ros::Time gripper_start_time_;

    // ====== Thread Safety ======
    std::mutex mutex_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "moveit_to_serial_bridge");
    MoveItToSerialBridge bridge;
    ros::spin();
    return 0;
}