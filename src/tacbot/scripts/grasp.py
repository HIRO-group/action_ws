#! /usr/bin/env python3
import franka_gripper.msg
import rospy
import sys
import actionlib
from std_msgs.msg import Float32MultiArray



class hiro_grasp:
    def __init__(self):
        rospy.init_node("grasp")
        self.open()

    def __set_open_grasp(self):
        self.__width = 0.03
        self.__inner = 0.01
        self.__outer = 0.01
        self.__speed = 0.05
        self.__force = 5

    def grasp(self):
        client = actionlib.SimpleActionClient('/franka_gripper/grasp', franka_gripper.msg.GraspAction)
        client.wait_for_server()
        goal = franka_gripper.msg.GraspGoal()
        goal.width = self.__width
        goal.epsilon.inner = self.__inner
        goal.epsilon.outer = self.__outer
        goal.speed = self.__speed
        goal.force = self.__force
        client.send_goal(goal)
        client.wait_for_result()
        return client.get_result()


    def home(self):
        client = actionlib.SimpleActionClient('/franka_gripper/homing', franka_gripper.msg.HomingAction)
        client.wait_for_server()
        goal = franka_gripper.msg.HomingGoal()
        # goal.width = self.__width
        # goal.epsilon.inner = self.__inner
        # goal.epsilon.outer = self.__outer
        # goal.speed = self.__speed
        # goal.force = self.__force
        client.send_goal(goal)
        client.wait_for_result()
        return client.get_result()

    def open(self):
        self.__set_open_grasp()
        # self.grasp()
    
    def set_grasp_width(self, width):
        self.__width = width

if __name__ == '__main__':
    try:
      hiro_g = hiro_grasp()
      result = hiro_g.home()
      print("Success: ", result.success)
      print("Error: ", result.error)
    except rospy.ROSInterruptException:
        print("error")


    try:
      hiro_g = hiro_grasp()
      result = hiro_g.grasp()
      print("Success: ", result.success)
      print("Error: ", result.error)
    except rospy.ROSInterruptException:
        print("error")