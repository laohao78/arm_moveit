#!/usr/bin/env python3

import rospy
from arm_serial_driver.msg import ArmSerialCommand
import sys

def main():
    rospy.init_node('test_arm_serial_command', anonymous=True)
    pub = rospy.Publisher('/arm_serial_command', ArmSerialCommand, queue_size=10)
    rate = rospy.Rate(50)  # 50 Hz

    # 默认关节值（弧度或你定义的单位）*********************************************************************************************
    default_positions = [
        6.0, # j1
        0.0, # j2
        0.0, # j3
        0.0, # j4
        0.0, # j5
        0.0, # j6--平动
        0.0, # j7
        0.0] # j8--夹爪
    #****************************************************************************************************************************

    # 如果命令行传入了 8 个数值，则使用它们
    if len(sys.argv) == 9:
        try:
            positions = [float(x) for x in sys.argv[1:9]]
            rospy.loginfo("Using command-line positions: %s", positions)
        except ValueError:
            rospy.logerr("Invalid number format in arguments. Using defaults.")
            positions = default_positions
    else:
        positions = default_positions
        rospy.loginfo("Using default positions (all zeros).")
        rospy.loginfo("Usage: rosrun arm_serial_driver test_arm_command.py j1 j2 j3 j4 j5 j6 j7 j8")

    msg = ArmSerialCommand()
    msg.position = positions

    rospy.loginfo("Publishing to /arm_serial_command... Press Ctrl+C to stop.")

    while not rospy.is_shutdown():
        # msg.header.stamp = rospy.Time.now()
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass