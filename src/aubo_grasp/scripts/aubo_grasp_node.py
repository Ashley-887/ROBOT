#!/usr/bin/env python
import rospy
import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

import rospy
import moveit_commander
from geometry_msgs.msg import PoseStamped
from version_pkg.msg import Objects  # 订阅视觉组发布的识别消息
import numpy as np
from gripper_control import GripperControl  # 导入GripperControl类


class AuboGraspNode:
    def __init__(self):
        # 初始化 ROS 节点
        rospy.init_node('aubo_grasp_node', anonymous=True)

        # 初始化 MoveIt
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander("manipulator")
        self.group.set_planning_time(10)

        # 订阅视觉组发布的物体识别信息
        rospy.Subscriber('/detections', Objects, self.object_callback)

        # 定义抓取的目标位姿
        self.target_pose = PoseStamped()

        # 设置初始位置
        self.home_pose = self.group.get_current_pose().pose

        # 初始化机械爪控制
        self.gripper_control = GripperControl(open_angle=0.2, close_angle=1.0)

        # 设置手眼标定的矩阵
        self.hand_eye_matrix = self.get_hand_eye_calibration()

        rospy.loginfo("AuboGraspNode initialized. Waiting for object detections...")

    def get_hand_eye_calibration(self):
        """
        手眼标定，返回一个4x4的变换矩阵，从相机坐标系转换到机械臂基座坐标系
        """
        hand_eye_matrix = np.array([[0.999, 0.001, 0.003, 0.12],
                                    [-0.001, 1.000, 0.002, 0.08],
                                    [-0.003, -0.002, 1.000, 0.25],
                                    [0, 0, 0, 1]])
        return hand_eye_matrix

    def object_callback(self, msg):
        """
        当接收到视觉组的识别信息时，转换物体坐标，并执行路径规划和抓取操作
        """
        if not msg.detections:
            rospy.logwarn("No objects detected.")
            return

        obj = msg.detections[0]
        obj_position_camera = np.array([obj.box[0], obj.box[1], obj.box[2], 1])

        # 将物体坐标转换到机械臂基座坐标系
        obj_position_base = np.dot(self.hand_eye_matrix, obj_position_camera)
        x, y, z = obj_position_base[:3]

        # 设置抓取的目标位姿
        self.target_pose.pose.position.x = x
        self.target_pose.pose.position.y = y
        self.target_pose.pose.position.z = z
        self.target_pose.pose.orientation.x = 0
        self.target_pose.pose.orientation.y = 0
        self.target_pose.pose.orientation.z = 0
        self.target_pose.pose.orientation.w = 1

        # 执行抓取
        self.execute_grasp()

    def execute_grasp(self, retries=3):
        """
        使用 MoveIt 执行路径规划并移动机械臂到目标位姿进行抓取，支持重试
        """
        success = False
        for attempt in range(retries):
            self.group.set_pose_target(self.target_pose)
            plan = self.group.go(wait=True)
            self.group.stop()

            if plan:
                rospy.loginfo(f"Grasp executed successfully on attempt {attempt + 1}.")
                success = True
                
                # 控制机械爪闭合
                self.gripper_control.close_gripper()
                rospy.sleep(1)
                
                break
            rospy.logwarn(f"Attempt {attempt + 1} failed to execute grasp.")
        
        if success:
            self.return_to_home()
            # 控制机械爪张开
            self.gripper_control.open_gripper()
        else:
            rospy.logwarn("Failed to execute grasp after several attempts.")

    def return_to_home(self):
        """
        抓取完成后返回初始位置
        """
        self.group.set_pose_target(self.home_pose)
        plan = self.group.go(wait=True)

        if plan:
            rospy.loginfo("Returned to home position.")
        else:
            rospy.logwarn("Failed to return to home position.")

if __name__ == '__main__':
    try:
        node = AuboGraspNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
