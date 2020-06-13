#!/usr/bin/env python
import roslib
roslib.load_manifest('faraday_controller')

import rospy
import actionlib
from std_msgs.msg import Float64
import trajectory_msgs.msg
import control_msgs.msg
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from control_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal, FollowJointTrajectoryAction, FollowJointTrajectoryGoal



class Joint:
        def __init__(self, motor_name):
            #arm_name should be b_arm or f_arm
            self.name = motor_name
            self.jta = actionlib.SimpleActionClient('/'+self.name+'_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
            rospy.loginfo('Waiting for joint trajectory action')
            self.jta.wait_for_server()
            rospy.loginfo('Found joint trajectory action!')


        def move_joint(self, angles):
            goal = FollowJointTrajectoryGoal()
            # char = self.name[0] #either 'f' or 'b'
            # goal.trajectory.joint_names = ['joint_1'+char, 'joint_2'+char,'joint_3'+char,'joint_4'+char,'joint_5'+char,'joint_7'+char,'joint_8'+char]
            goal.trajectory.joint_names = ['joint1', 'joint2','joint3','joint4','joint5']
            point = JointTrajectoryPoint()
            point.positions = angles
            point.time_from_start = rospy.Duration(1.0)
            goal.trajectory.points.append(point)
            self.jta.send_goal_and_wait(goal)
            rospy.loginfo('Just finish the goal!')


def main():
            arm = Joint('fy_pm')
            #motor initial position
            arm.move_joint([87.1446, 85.6595, 85.6595, 87.1446, 0.0])
            # arm.move_joint([140.8, 170.8, 165.8, 140.8, 9.2, 113.5, 0])
            # arm.move_joint([0,3.14,6.28,2.1,2.2,2.3,3.4])
            # arm.move_joint([0,6.28,6.28,1.0,6.28,6.28,6.28])
            # arm.move_joint([0,3.14,3.14,0,0,0,0])
            # arm.move_joint([0,0,3.14,6.28,0,0,0])
            # arm.move_joint([0,6.28,3.14,0,0,0,0])
            # arm.move_joint([0,3.14,3.14,6.28,0,0,0])

if __name__ == '__main__':
      rospy.init_node('joint_position_tester')
      
      main()