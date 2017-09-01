#!/usr/bin/env python

import rospy
import mavros
import sensor_msgs
import yaml
#from mavros.msg import *
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix

#global variables
latitude = 0.0
longitude = 0.0
altitude = 0.0
last_waypoint = False

def waypoint_callback(data):
	global last_waypoint
	print("\n----------\nwaypoint_callback")
	rospy.loginfo("Got waypoint: %s", data)
#	print(len(data.waypoints))
#	print("INTERESTING DATA: " + repr(len(data.waypoints)))
	if len(data.waypoints) != 0:							#if waypoint list is not empty
		rospy.loginfo("is_current: %s", data.waypoints[len(data.waypoints)-1].is_current)
		last_waypoint = data.waypoints[len(data.waypoints)-1].is_current	#checks status of "is_current" for last waypoint
#	print(data)


def globalPosition_callback(data):
	#print("\n----------\nglobalPosition_callback")
	global latitude
	global longitude
	global altitude
	latitude = data.latitude
	longitude = data.longitude
	altitude = data.altitude


def main():
	rospy.init_node('wayPoint')
	rospy.Subscriber("/mavros/mission/waypoints", WaypointList, waypoint_callback)
	rospy.Subscriber("/mavros/global_position/raw/fix", NavSatFix, globalPosition_callback)

	#Clearing waypoints
	print("\n----------CLEARING----------")
	rospy.wait_for_service("/mavros/mission/clear")
	print("Clearing Waypoints!!!")
	waypoint_clear = rospy.ServiceProxy("/mavros/mission/clear", WaypointClear)
	resp = waypoint_clear()
	print(resp)
	rospy.sleep(5)

	#Call waypoints_pull
	print("\n----------PULLING----------")
	rospy.wait_for_service("/mavros/mission/pull")
	print("Calling Waypoint_pull Service")
	waypoint_pull = rospy.ServiceProxy("/mavros/mission/pull", WaypointPull)
	resp = waypoint_pull()
	print(resp)
	rospy.sleep(5)
	
	#Arming
	print("\n----------ARMING----------")
	rospy.wait_for_service("/mavros/cmd/arming")
	print("Arming UAV!!!")
	uav_arm = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
	resp = uav_arm(1)
	print(resp)
	rospy.sleep(5)
	
	#Sending waypoints_push
	print("\n----------PUSHING----------")
	print("Waiting for MAVROS service...")
	rospy.wait_for_service("/mavros/mission/push")
	
	waypoints = [
		Waypoint(frame = 3, command = 22, is_current = True, autocontinue = True, param1 = 5, x_lat = 47.3975922, y_long = 8.5455939, z_alt = 5),
		Waypoint(frame = 3, command = 16, is_current = False, autocontinue = True, param1 = 5, x_lat = 47.3979922, y_long = 8.5455939, z_alt = 10),
		Waypoint(frame = 3, command = 21, is_current = False, autocontinue = True, param1 = 5, x_lat = 47.3989922, y_long = 8.5455939, z_alt = 15)
	]
	
	waypoint_push = rospy.ServiceProxy("/mavros/mission/push", WaypointPush)
	resp = waypoint_push(waypoints)
	print(resp)
	rospy.sleep(5)
	
#	#Call waypoints_pull
#	print("\n----------PULLING----------")
#	rospy.wait_for_service("/mavros/mission/pull")
#	print("Calling Waypoint_pull Service")
#	waypoint_pull = rospy.ServiceProxy("/mavros/mission/pull", WaypointPull)
#	resp = waypoint_pull()
#	print(resp)
#	rospy.sleep(5)
	
	while True:						#waits for last_waypoint in previous WaypointList to be visited
		rospy.sleep(2)
		print("WAITING11111111111111111111111111111111")
		if last_waypoint == True:			#if last_waypoint is in the process of being visited
			while True:
				rospy.sleep(2)
				print("WAITING222222222222222222222222222222222")
				if last_waypoint == False:	#if last_waypoint has been visited (due to previous constraint)
					break
			break
	
#	rospy.sleep(10)						#allows for the finishing of the land proceedure
	#Clearing waypoints
	print("\n----------CLEARING----------")
	rospy.wait_for_service("/mavros/mission/clear")
	print("Clearing Waypoints!!!")
	waypoint_clear = rospy.ServiceProxy("/mavros/mission/clear", WaypointClear)
	resp = waypoint_clear()
	print(resp)
	rospy.sleep(5)
	#Call waypoints_pull
	print("\n----------PULLING----------")
	rospy.wait_for_service("/mavros/mission/pull")
	print("Calling Waypoint_pull Service")
	waypoint_pull = rospy.ServiceProxy("/mavros/mission/pull", WaypointPull)
	resp = waypoint_pull()
	print(resp)
	rospy.sleep(5)

	while True:
		rospy.sleep(2)
		print("WAITING3333333333333333333333333333333")
		#print(" lat " + repr(latitude) + " long " + repr(longitude) + " alt " + repr(altitude))
		latlongalt = (latitude-47.39899)+(longitude-8.54559)+(altitude-488)		#checks for total difference is less than 0.0001
		if latlongalt < 0.0001:
			rospy.wait_for_service("/mavros/mission/push")
			resp = waypoint_push(waypoints)
			waypoints = [
				Waypoint(frame = 3, command = 22, is_current = True, autocontinue = True, param1 = 5, x_lat = 47.3975922, y_long = 8.5455939, z_alt = 20),
				Waypoint(frame = 3, command = 21, is_current = True, autocontinue = True, param1 = 5, x_lat = 47.3975922, y_long = 8.5455939, z_alt = 20),
			]
			resp = waypoint_push(waypoints)
			print(resp)
			rospy.sleep(5)
			break
	#Arming
	print("\n----------ARMING----------")
	rospy.wait_for_service("/mavros/cmd/arming")
	print("Arming UAV!!!")
	uav_arm = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
	resp = uav_arm(1)
	print(resp)
	rospy.sleep(5)

	while True:						#waits for last_waypoint in previous WaypointList to be visited
		rospy.sleep(2)
		print("WAITING4444444444444444444444444444")
		if last_waypoint == True:			#if last_waypoint is in the process of being visited
			while True:
				rospy.sleep(2)
				print("WAITING555555555555555555555555555555555555555")
				if last_waypoint == False:	#if last_waypoint has been visited (due to previous constraint)
					break
			break

	
	print("fin")
	rospy.spin()

	


if __name__ == '__main__':
	main()

