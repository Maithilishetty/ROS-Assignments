#!/usr/bin/env python

from lablet_ap101_msgs.srv import LinearPlanner, LinearPlannerRequest, LinearPlannerResponse
import rospy
import math
from math import pi
from lablet_ap101_msgs.msg import SensorValue, Response, Request
from geometry_msgs.msg import Pose2D

def process_request(req):

	#Assign all the values and variables
	res = LinearPlannerResponse()
	a = req.msg.initial_pose.x
	b = req.msg.initial_pose.y
	dir = req.msg.direction
	dir_dict = {'N': 0, 'NE':-pi/4, 'E':-pi/2, 'SE':-3*pi/4, 'S':pi, 'SW':3*pi/4, 'W':pi/2, 'NW':pi/4}
	d = req.msg.distance
	num = req.msg.plan_resolution

	theta = dir_dict[dir]
	goal_pose_y = b-math.sqrt(math.pow(d,2)/math.pow(1+math.tan(theta),2))
	goal_pose_x = a - math.tan(theta)*(b-goal_pose_y)

	if num<3:
		res.msg.path_waypoints = []
		res.msg.plan_status = res.PLAN_INVALID
	else:
		#Perform Linear Interpolation
		step = (goal_pose_y-b)/num
		step_angle = (pi - abs(abs(0 - theta) - pi))/num
		new_angle = 0
		new_y = 0
		new_x = 0
		new_angle = 0
		new_points = []
		res.msg.path_waypoints = []

		for i in range(0, num):
			pose = Pose2D()
			new_y = b + step*i
			new_x = a + (new_y-b)*((goal_pose_x-a)/(goal_pose_y-b))
			new_angle = 0 + step_angle*i
			pose.x = new_x
			pose.y = new_y
			pose.theta = new_angle
			res.msg.path_waypoints.append(pose)

		res.msg.plan_status = res.PLAN_SUCCESS

	return res


def LinearPlanner_server():

	rospy.init_node('linear_planner',anonymous=False)

	service = rospy.Service('LinearPlanner_service', LinearPlanner, process_request)

	rospy.loginfo('Server is now running...')
	rospy.spin()

if __name__ == "__main__":
	
	LinearPlanner_server()
