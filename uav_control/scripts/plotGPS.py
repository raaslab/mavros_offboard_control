#!/usr/bin/env python

import rospy
import mavros
import sensor_msgs
import yaml
import utm
import time
import matplotlib.pyplot as plt
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from std_msgs.msg import String
from std_msgs.msg import Int64
from sensor_msgs.msg import NavSatFix

#global variables
latitude = 0.0
longitude = 0.0
altitude = 0.0
last_waypoint = False
tolerance = 0.00015
ugv_ready = "0"

def waypoint_callback(data):
	print("\n----------waypoint_callback----------")
	global last_waypoint
	rospy.loginfo("Got waypoint: %s", data)
	if len(data.waypoints) != 0:							# If waypoint list is not empty
		rospy.loginfo("is_current: %s", data.waypoints[len(data.waypoints)-1].is_current)
		last_waypoint = data.waypoints[len(data.waypoints)-1].is_current	# Checks status of "is_current" for last waypoint

def globalPosition_callback(data):
	# print("\n----------globalPosition_callback----------")
	global latitude
	global longitude
	global altitude
	latitude = data.latitude
	longitude = data.longitude
	altitude = data.altitude

def ready_callback(data):
	# rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
	global ugv_ready
	ugv_ready = data.data

def pushingWaypoints(poi):
	print("\n----------pushingWaypoints----------")
	rospy.wait_for_service("/mavros/mission/push")
	waypoint_push = rospy.ServiceProxy("/mavros/mission/push", WaypointPush)
	resp = waypoint_push(0,poi)
	rospy.sleep(5)
	return

def takeoff_call(lat, long, alt):
	print("\n----------takeoff_call----------")
	takeoff = rospy.ServiceProxy("/mavros/cmd/takeoff", CommandTOL)
	resp = takeoff(0,0,lat,long, alt)
	rospy.sleep(5)
	return

def plotData(xData, yData, axis):
	plt.plot(xData, yData, 'ro')
	if axis == 0:
		return
	else:
		plt.axis(axis)
	plt.show()

def main():
	rospy.init_node('gpsPlotter')
	rospy.Subscriber("/mavros/mission/waypoints", WaypointList, waypoint_callback)
	rospy.Subscriber("/mavros/global_position/raw/fix", NavSatFix, globalPosition_callback)
	# rospy.Subscriber("/mavros/ugv/ready", Int64, ready_callback)	
	# readyBit = rospy.Publisher("/mavros/uav/ready", Int64, queue_size=10) # Flag topic
	# readyBit.publish(0)
	# clear_pull()

	
	xData = []
	yData = []
	counter = 0
	while True:
		[xDataTemp, yDataTemp, rando, rando1] = utm.from_latlon(latitude, longitude)
		xData.append(xDataTemp)
		yData.append(yDataTemp)
		time.sleep(1)
		counter = counter + 1
		if counter == 400:
			break

	plotData(xData, yData, 0)

	# DONE
	print("Finished")
	rospy.spin()


if __name__ == '__main__':
	main()
