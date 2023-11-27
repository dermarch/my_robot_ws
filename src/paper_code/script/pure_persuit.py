#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import csv
import math

from geometry_msgs.msg import Quaternion, PoseStamped, TwistStamped, Twist

from styx_msgs.msg import Lane, Waypoint

from gazebo_msgs.msg import ModelStates

import tf
import rospy

HORIZON = 6.0 											#往前看多远算成我们的目标点

class PurePersuit:
	def __init__(self):
		rospy.init_node('pure_persuit', log_level=rospy.DEBUG)

		rospy.Subscriber('/smart/rear_pose', PoseStamped, self.pose_cb, queue_size = 1)   #获取车辆后轴中心点的位置、速度、局部路径点
		rospy.Subscriber('/smart/velocity', TwistStamped, self.vel_cb, queue_size = 1)
		rospy.Subscriber('/final_waypoints', Lane, self.lane_cb, queue_size = 1)

		self.twist_pub = rospy.Publisher('/smart/cmd_vel', Twist, queue_size = 1)	  #发布速度指令

		#定义三个变量
		self.currentPose = None
		self.currentVelocity = None
		self.currentWaypoints = None

		self.loop()

	def loop(self):
		rate = rospy.Rate(20)
		rospy.logwarn("pure persuit starts")

		while not rospy.is_shutdown():
			if self.currentPose and self.currentVelocity and self.currentWaypoints:  #当我三个指令都收到后
				twistCommand = self.calculateTwistCommand()    #计算Twist
				self.twist_pub.publish(twistCommand)
			rate.sleep()

	def pose_cb(self,data):
		self.currentPose = data

	def vel_cb(self,data):
		self.currentVelocity = data

	def lane_cb(self,data):
		self.currentWaypoints = data

	def calculateTwistCommand(self):
		lad = 0.0 #look ahead distance accumulator      前视距离      里程计
		targetIndex = len(self.currentWaypoints.waypoints) - 1
		for i in range(len(self.currentWaypoints.waypoints)):
			if((i+1) < len(self.currentWaypoints.waypoints)):
				this_x = self.currentWaypoints.waypoints[i].pose.pose.position.x
				this_y = self.currentWaypoints.waypoints[i].pose.pose.position.y
				next_x = self.currentWaypoints.waypoints[i+1].pose.pose.position.x
				next_y = self.currentWaypoints.waypoints[i+1].pose.pose.position.y
				lad = lad + math.hypot(next_x - this_x, next_y - this_y)
				if(lad > HORIZON):      #当累加的前视距离>HORIZON 然后选取目标点 
					targetIndex = i+1
					break


		targetWaypoint = self.currentWaypoints.waypoints[targetIndex]         #将目标点读取进取

		targetSpeed = self.currentWaypoints.waypoints[0].twist.twist.linear.x  #速度信息

		targetX = targetWaypoint.pose.pose.position.x    #目标点和当前的XY
		targetY = targetWaypoint.pose.pose.position.y		
		currentX = self.currentPose.pose.position.x
		currentY = self.currentPose.pose.position.y
		#get vehicle yaw angle  四元素的转换
		quanternion = (self.currentPose.pose.orientation.x, self.currentPose.pose.orientation.y, self.currentPose.pose.orientation.z, self.currentPose.pose.orientation.w)
		euler = tf.transformations.euler_from_quaternion(quanternion)
		yaw = euler[2]    #得到了车辆的航向角
		#get angle difference
		alpha = math.atan2(targetY - currentY, targetX - currentX) - yaw    #目标点与后轮中心点的连线与航向角之间的夹角，纯跟踪
		l = math.sqrt(math.pow(currentX - targetX, 2) + math.pow(currentY - targetY, 2))  #目标点与后轮中心点的距离
		if(l > 0.5):
			theta = math.atan(2 * 1.868 * math.sin(alpha) / l)        #正常情况下纯跟踪的前轮转角
			# #get twist command
			twistCmd = Twist()
			twistCmd.linear.x = targetSpeed
			twistCmd.angular.z = theta 
		else:
			twistCmd = Twist()						#距离特别小的时候设置速度为0；转向为0
			twistCmd.linear.x = 0
			twistCmd.angular.z = 0

		return twistCmd


if __name__ == '__main__':
    try:
        PurePersuit()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start motion control node.')

