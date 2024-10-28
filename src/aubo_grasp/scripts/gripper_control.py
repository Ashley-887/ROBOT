#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64  # 假设机械爪接受Float64类型的开合角度控制消息

class GripperControl:
    def __init__(self, open_angle=0.0, close_angle=1.0):
        # 初始化机械爪的开闭角度参数
        self.open_angle = open_angle
        self.close_angle = close_angle
        self.gripper_pub = rospy.Publisher('/gripper_command', Float64, queue_size=10)
        rospy.loginfo("GripperControl initialized.")

    def open_gripper(self):
        """
        控制机械爪张开
        """
        self.gripper_pub.publish(Float64(self.open_angle))
        rospy.loginfo("Gripper opened.")

    def close_gripper(self):
        """
        控制机械爪闭合
        """
        self.gripper_pub.publish(Float64(self.close_angle))
        rospy.loginfo("Gripper closed.")

# 仅在独立运行时执行抓取测试
if __name__ == '__main__':
    try:
        rospy.init_node('gripper_control_test')
        gripper = GripperControl(open_angle=0.2, close_angle=1.0)

        # 测试开合
        rospy.sleep(1)
        gripper.open_gripper()
        rospy.sleep(2)
        gripper.close_gripper()
    except rospy.ROSInterruptException:
        pass
