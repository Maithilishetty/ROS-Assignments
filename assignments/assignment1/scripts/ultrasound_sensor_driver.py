#!/usr/bin/env python


import rospy
import numpy as np
from lablet_ap101_msgs.msg import SensorValue


def sensor_info_publisher():

	#Initialize the Publisher and Node
	sensor_pub = rospy.Publisher('ultrasound_sensor_values',SensorValue,queue_size=10)
	rospy.init_node('ultrasound_sensor_driver', anonymous = True)

	#Set the rate to 20 Hz
	rate = rospy.Rate(20)

	#Assign it to the custom message
	ultrasound_sensor_values = SensorValue()

	while not rospy.is_shutdown():
			raw_sensor_values = []
		 	count = 5

			for i in range(0,count):
				a = np.random.randint(16384,dtype=np.uint16)
				raw_sensor_values.append(a)

			rospy.loginfo(raw_sensor_values)


			sensor_value_validity = []
			n = 14

			#Check sensor validity
			for i in range(0,count):
				x = (raw_sensor_values[i] >> (n-1) and 1)
				if x == 0:
					sensor_value_validity.append('valid')
				else:
					sensor_value_validity.append('invalid')

			rospy.loginfo(sensor_value_validity)

			sensor_distances = []
			#value = rospy.get_param("sensor_resolution")
			#sensor_value = 0.5
			value = 0.5 #Sensor Resolution

			#Convert to m
			for i in range(0,count):
				if sensor_value_validity[i]=='valid':
					sensor_distances.append(raw_sensor_values[i]*value*0.001)
				else:
					sensor_distances.append(np.Inf)


			rospy.loginfo(sensor_distances)
			direction_values=[]
			count_1 = 8
			n = 14

			#Generate random 8 direction values which are valid
			for i in range(0,count_1):
				b = np.random.randint(16384,dtype=np.uint16)
				if (b>>(n-1) and 1)!=0:
					b = b & (~(1 << (n - 1)))
				direction_values.append(b)

			rospy.loginfo(direction_values)

			direction_value_validity = []
			n = 14

			#Verify sensor validity
			for i in range(0,count_1):
				y = (direction_values[i] >> (n-1) and 1)
				if y == 0:
					direction_value_validity.append('valid')
				else:
					direction_value_validity.append('invalid')

			rospy.loginfo(direction_value_validity)

			#Publish to Node 
			ultrasound_sensor_values.sensor_data.header.stamp = rospy.Time.now()

			ultrasound_sensor_values.sensor_data.radiation_type = ultrasound_sensor_values.sensor_data.ULTRASOUND
			ultrasound_sensor_values.sensor_data.min_range= 0.01
			ultrasound_sensor_values.sensor_data.max_range = 2.00

			ultrasound_sensor_values.sensor_raw_value = direction_values
			rospy.loginfo(ultrasound_sensor_values.sensor_raw_value)

			rospy.loginfo(ultrasound_sensor_values)
			sensor_pub.publish(ultrasound_sensor_values)
			rate.sleep()



if __name__ == '__main__':

	sensor_info_publisher()
