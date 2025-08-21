#!/usr/bin/env python

import rospy
import numpy as np
from lablet_ap101_msgs.msg import SensorValue, Response, Request
from lablet_ap101_msgs.srv import LinearPlanner, LinearPlannerRequest, LinearPlannerResponse

def sensor_info_callback(data):

	#Obtain the max value and print the direction message
	dir = ['N', 'NE', 'E', 'SE', 'S', 'SW', 'W', 'NW']
	x = data.sensor_raw_value
	i = x.index(max(x))
	rospy.loginfo('Move towards %s', dir[i])

	#Call the request message and assign values
	req = Request()

	req.direction = dir[i]
	req.distance = max(data.sensor_raw_value)*0.5*0.001
	req.initial_pose.x = 0
	req.initial_pose.y = 0
	req.initial_pose.theta = 0
	req.plan_resolution = 10

	#Call the client
	service_response = LinearPlanner_client(req)
	rospy.loginfo(service_response)

def sensor_info_subscriber():

	#Subsrcribing to the publisher defined in 'ultrasound_sensor_driver'
	rospy.init_node('collision_avoidance_node', anonymous = False)
	rospy.Subscriber('ultrasound_sensor_values',SensorValue,sensor_info_callback)

	rospy.spin()

def LinearPlanner_client(req):

	#makes a server call and returns the values 
	rospy.loginfo('Starting the client service...')
	rospy.wait_for_service('LinearPlanner_service')
	try:
		LinearPlanner_service = rospy.ServiceProxy('LinearPlanner_service', LinearPlanner)
		service_response = LinearPlanner_service(req)
		return service_response

	except rospy.ServiceException, e:
		print "Servive call failed: %s"%e

if __name__ == '__main__':

	sensor_info_subscriber()
