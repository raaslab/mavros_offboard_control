#!/usr/bin/env python

import rospy
import mavros
import sensor_msgs
import yaml
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

def waiting_ugv(lat, long, alt):
	print("\n----------waiting_ugv----------")
	while True:
		checker = 1
		if checker == 1:
		# if ugv_ready == 1:
			waypoints = [
			Waypoint(frame = 3, command = 21, is_current = 0, autocontinue = True, param1 = 5, x_lat = lat, y_long = long, z_alt = alt),
			Waypoint(frame = 3, command = 21, is_current = 1, autocontinue = True, param1 = 5, x_lat = lat, y_long = long, z_alt = alt)
			]
			waypoint_push = rospy.ServiceProxy("/mavros/mission/push", WaypointPush)
			resp = waypoint_push(0, waypoints)
			rospy.sleep(5)			
			return

def clear_pull():
	print("\n----------clear_pull----------")
	# Clearing waypoints
	rospy.wait_for_service("/mavros/mission/clear")
	waypoint_clear = rospy.ServiceProxy("/mavros/mission/clear", WaypointClear)
	resp = waypoint_clear()
	rospy.sleep(5)
	# Call waypoints_pull
	rospy.wait_for_service("/mavros/mission/pull")
	waypoint_pull = rospy.ServiceProxy("/mavros/mission/pull", WaypointPull)
	resp = waypoint_pull()
	rospy.sleep(5)
	return

def finishWaypoints(lat, long, pub):
	print("\n----------finishwaypoints----------")
	while True:						# Waits for last_waypoint in previous WaypointList to be visited
		rospy.sleep(2)
		# Waiting for last_waypoint to be true
		if last_waypoint == True:			# If last_waypoint is in the process of being visited
			while True:
				rospy.sleep(15)
				# Waiting for last_waypoint to be false
			#	if abs(latitude-(lat))<tolerance and abs(longitude-(long))<tolerance:
				# if last_waypoint == False:	# If last_waypoint has been visited (due to previous constraint)
					#pub.publish(1)
			#		break
			break
	return

def armingCall():
	print("\n----------armingCall----------")
	rospy.wait_for_service("mavros/cmd/arming")
	uav_arm = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
	resp = uav_arm(1)
	rospy.sleep(5)

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

def switch_modes(current_mode, next_mode, delay): # current_mode: int, next_mode: str (http://docs.ros.org/jade/api/mavros_msgs/html/srv/SetMode.html)
	print("\n----------switch_modes----------")
	rospy.wait_for_service("/mavros/set_mode")
	modes = rospy.ServiceProxy("/mavros/set_mode", SetMode)
	resp = modes(current_mode, next_mode)
	rospy.sleep(delay)
	return

def takeoff_waypoint_land(waypoints, takeoff_point, land_point, readyBit, last_point):
	switch_modes(0, "stabilize", 5)
	armingCall()
	switch_modes(0, "guided", 5)
	takeoff_call(takeoff_point[0], takeoff_point[1], 10)
	pushingWaypoints(waypoints) # Pushes waypoints to UAV
	switch_modes(0, "auto", 5)
	finishWaypoints(land_point[0], land_point[1], readyBit)	# Checks if waypoints are finished
	clear_pull() # Logistic house keeping
	if last_point == 0:
		waiting_ugv(land_point[0], land_point[1], 0)	# Checks if ugv is at lat long
	else:
		waypoints = [
		Waypoint(frame = 3, command = 21, is_current = 0, autocontinue = True, param1 = 5, x_lat = land_point[0], y_long = land_point[1], z_alt = 0),
		Waypoint(frame = 3, command = 21, is_current = 1, autocontinue = True, param1 = 5, x_lat = land_point[0], y_long = land_point[1], z_alt = 0)
		]
		waypoint_push = rospy.ServiceProxy("/mavros/mission/push", WaypointPush)
		resp = waypoint_push(0, waypoints)
		rospy.sleep(5)
	switch_modes(0, "guided", 1)
	switch_modes(0, "auto", 5)
	return

def main():
	rospy.init_node('uav_node')
	rospy.Subscriber("/mavros/mission/waypoints", WaypointList, waypoint_callback)
	rospy.Subscriber("/mavros/global_position/raw/fix", NavSatFix, globalPosition_callback)
	rospy.Subscriber("/mavros/ugv/ready", Int64, ready_callback)	
	readyBit = rospy.Publisher("/mavros/uav/ready", Int64, queue_size=10) # Flag topic
	readyBit.publish(0)
	# clear_pull()
	
	print("Which set of waypoints?")
	waypoint_section = int(input())

	waypoints1 = [	# Sending waypoints_push
		Waypoint(frame = 3, command = 16, is_current = 1, autocontinue = True, param1 = 5, x_lat = 37.198292, y_long = -80.578233, z_alt = 20),
		Waypoint(frame = 3, command = 16, is_current = 0, autocontinue = True, param1 = 5, x_lat = 37.198292, y_long = -80.578233, z_alt = 20),
		Waypoint(frame = 3, command = 16, is_current = 0, autocontinue = True, param1 = 5, x_lat = 37.198702, y_long = -80.578812, z_alt = 20),
		Waypoint(frame = 3, command = 16, is_current = 0, autocontinue = True, param1 = 5, x_lat = 37.198292, y_long = -80.578233, z_alt = 20)
	]
	takeoff1 = [37.198292, -80.578233]
	land1 = [37.198292, -80.578233]

	waypoints2 = [
		Waypoint(frame = 3, command = 16, is_current = 1, autocontinue = True, param1 = 5, x_lat = 37.198292, y_long = -80.578233, z_alt = 20),
		Waypoint(frame = 3, command = 16, is_current = 0, autocontinue = True, param1 = 5, x_lat = 37.198292, y_long = -80.578233, z_alt = 20),
		Waypoint(frame = 3, command = 16, is_current = 0, autocontinue = True, param1 = 5, x_lat = 37.199155, y_long = -80.579478, z_alt = 20),
		Waypoint(frame = 3, command = 16, is_current = 0, autocontinue = True, param1 = 5, x_lat = 37.199242, y_long = -80.579290, z_alt = 20),
		Waypoint(frame = 3, command = 16, is_current = 0, autocontinue = True, param1 = 5, x_lat = 37.198155, y_long = -80.580722, z_alt = 20),
		Waypoint(frame = 3, command = 16, is_current = 0, autocontinue = True, param1 = 5, x_lat = 37.198813, y_long = -80.580078, z_alt = 20)
		]
	takeoff2 = [37.198292, -80.578233]
	land2 = [37.198813, -80.580078]

	waypoints3 = [
		Waypoint(frame = 3, command = 16, is_current = 1, autocontinue = True, param1 = 5, x_lat = 37.198813, y_long = -80.580078, z_alt = 20),
		Waypoint(frame = 3, command = 16, is_current = 0, autocontinue = True, param1 = 5, x_lat = 37.198813, y_long = -80.580078, z_alt = 20),
		Waypoint(frame = 3, command = 16, is_current = 0, autocontinue = True, param1 = 5, x_lat = 37.198523, y_long = -80.580363, z_alt = 20),
		Waypoint(frame = 3, command = 16, is_current = 0, autocontinue = True, param1 = 5, x_lat = 37.198745, y_long = -80.580127, z_alt = 20),
		Waypoint(frame = 3, command = 16, is_current = 0, autocontinue = True, param1 = 5, x_lat = 37.198523, y_long = -80.580363, z_alt = 20),
		Waypoint(frame = 3, command = 16, is_current = 0, autocontinue = True, param1 = 5, x_lat = 37.198745, y_long = -80.580127, z_alt = 20),
		Waypoint(frame = 3, command = 16, is_current = 0, autocontinue = True, param1 = 5, x_lat = 37.19523, y_long = -80.580363, z_alt = 20),
		Waypoint(frame = 3, command = 16, is_current = 0, autocontinue = True, param1 = 5, x_lat = 37.198813, y_long = -80.580078, z_alt = 20)
	]
	takeoff3 = [37.198813,-80.580078]
	land3 = [37.198813,-80.580078]

	waypoints4 = [
		Waypoint(frame = 3, command = 16, is_current = 1, autocontinue = True, param1 = 5, x_lat = 37.198813, y_long = -80.580078, z_alt = 20),
		Waypoint(frame = 3, command = 16, is_current = 0, autocontinue = True, param1 = 5, x_lat = 37.198813, y_long = -80.580078, z_alt = 20),
		Waypoint(frame = 3, command = 16, is_current = 0, autocontinue = True, param1 = 5, x_lat = 37.199242, y_long = -80.579290, z_alt = 20),
		Waypoint(frame = 3, command = 16, is_current = 0, autocontinue = True, param1 = 5, x_lat = 37.198155, y_long = -80.580722, z_alt = 20),
		Waypoint(frame = 3, command = 16, is_current = 0, autocontinue = True, param1 = 5, x_lat = 37.199155, y_long = -80.579478, z_alt = 20),
		Waypoint(frame = 3, command = 16, is_current = 0, autocontinue = True, param1 = 5, x_lat = 37.198292, y_long = -80.578233, z_alt = 20)
	]
	takeoff4 = [37.198813,-80.580078]
	land4 = [37.198813,-80.580078]

	if waypoint_section == 1:
		while True:
			rospy.sleep(2)
			# print("Waiting for UAV to be close to next takeoff point")
			# if abs(latitude-takeoff1[0])<tolerance and abs(longitude-takeoff1[1])<tolerance:
			# readyBit.publish(0)
			takeoff_waypoint_land(waypoints1, takeoff1, land1, readyBit,1)
			break
	elif waypoint_section == 2:
		while True:
			rospy.sleep(2)
			print("Waiting for UAV to be close to next takeoff point")
			# if abs(latitude-takeoff2[0])<tolerance and abs(longitude-takeoff2[1])<tolerance:
			raw_input()			
			# readyBit.publish(0)
			takeoff_waypoint_land(waypoints2, takeoff2, land2, readyBit,0)
			break
	elif waypoint_section == 3:
		while True:
			rospy.sleep(2)
			print("Waiting for UAV to be close to next takeoff point")
			# if abs(latitude-takeoff3[0])<tolerance and abs(longitude-takeoff3[1])<tolerance:
			raw_input()			
			# readyBit.publish(0)
			takeoff_waypoint_land(waypoints3, takeoff3, land3, readyBit,0)
			break
	elif waypoint_section == 4:
		while True:
			rospy.sleep(2)
			print("Waiting for UAV to be close to next takeoff point")
			# if abs(latitude-takeoff4[0])<tolerance and abs(longitude-takeoff4[1])<tolerance:
			raw_input()			
			# readyBit.publish(0)
			takeoff_waypoint_land(waypoints4, takeoff4, land4, readyBit,1)
			break
	else:
		print("You inputed a wrong number. Try again.")

	# DONE
	print("Finished")
	rospy.spin()


if __name__ == '__main__':
	main()
