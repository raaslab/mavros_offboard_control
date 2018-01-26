#!/usr/bin/env python

import rospy
import mavros
import sensor_msgs
import yaml
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix

#global variables
latitude = 0.0
longitude = 0.0
altitude = 0.0
last_waypoint = False
tolerance = 0.00005

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

def waiting_ugv(lat, long, alt):
	print("\n----------waiting_ugv----------")
	while True:
		# TODO: add listener to the UGV flag here
		# checker = UGV publisher
		checker = 1
		if checker == 1:
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

def finishWaypoints(lat, long):
	print("\n----------finishwaypoints----------")
	while True:						# Waits for last_waypoint in previous WaypointList to be visited
		rospy.sleep(2)
		# Waiting for last_waypoint to be true
		if last_waypoint == True:			# If last_waypoint is in the process of being visited
			while True:
				rospy.sleep(2)
				# Waiting for last_waypoint to be false
				if abs(latitude-(lat))<tolerance and abs(longitude-(long))<tolerance:
				# if last_waypoint == False:	# If last_waypoint has been visited (due to previous constraint)
					break
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

def main():
	rospy.init_node('wayPoint')
	rospy.Subscriber("/mavros/mission/waypoints", WaypointList, waypoint_callback)
	rospy.Subscriber("/mavros/global_position/raw/fix", NavSatFix, globalPosition_callback)
	readyBit = rospy.Publisher("/mavros/ugv/ready", String, queue_size=10) # Flag topic
    
	clear_pull()
	armingCall()
	switch_modes(0, "guided", 5)
	takeoff_call(37.196749, -80.580270, 10)
	waypoints = [	# Sending waypoints_push
		Waypoint(frame = 3, command = 16, is_current = 1, autocontinue = True, param1 = 5, x_lat = 37.196749, y_long = -80.580270, z_alt = 10),
		Waypoint(frame = 3, command = 16, is_current = 0, autocontinue = True, param1 = 5, x_lat = 37.196749, y_long = -80.580270, z_alt = 10),
		Waypoint(frame = 3, command = 16, is_current = 0, autocontinue = True, param1 = 5, x_lat = 37.196828, y_long = -80.580235, z_alt = 10),
		Waypoint(frame = 3, command = 16, is_current = 0, autocontinue = True, param1 = 5, x_lat = 37.197043, y_long = -80.580264, z_alt = 10),
		Waypoint(frame = 3, command = 16, is_current = 0, autocontinue = True, param1 = 5, x_lat = 37.197191, y_long = -80.580355, z_alt = 10),
		Waypoint(frame = 3, command = 16, is_current = 0, autocontinue = True, param1 = 5, x_lat = 37.197376, y_long = -80.580608, z_alt = 10),
		Waypoint(frame = 3, command = 16, is_current = 0, autocontinue = True, param1 = 5, x_lat = 37.197532, y_long = -80.580561, z_alt = 10),
		Waypoint(frame = 3, command = 16, is_current = 0, autocontinue = True, param1 = 5, x_lat = 37.197623, y_long = -80.580627, z_alt = 10),
		Waypoint(frame = 3, command = 16, is_current = 0, autocontinue = True, param1 = 5, x_lat = 37.197730, y_long = -80.580546, z_alt = 10),
	]
	pushingWaypoints(waypoints) # Pushes waypoints to UAV
	switch_modes(0, "auto", 5)
	finishWaypoints(37.197730, -80.580546)	# Checks if waypoints are finished
	clear_pull() # Logistic house keeping
	waiting_ugv(37.197730, -80.580546, 0)	# Checks if ugv is at lat long
	swtich_modes(0, "guided", 1)
	swtich_modes(0, "auto", 5)

	while True:
		rospy.sleep(2)
		print("Waiting for UAV to be close to next takeoff point")
		if abs(latitude-(37.197779))<tolerance and abs(longitude-(-80.580670))<tolerance:
			swtich_modes(0, "stabalize", 5)
			armingCall()
			switch_modes(216, "guided", 5)
			takeoff_call(37.197779, -80.580670, 10)
			waypoints = [
				Waypoint(frame = 3, command = 16, is_current = 1, autocontinue = True, param1 = 5, x_lat = 37.197779, y_long = -80.580670, z_alt = 10),
				Waypoint(frame = 3, command = 16, is_current = 0, autocontinue = True, param1 = 5, x_lat = 37.197779, y_long = -80.580670, z_alt = 10),
				Waypoint(frame = 3, command = 16, is_current = 0, autocontinue = True, param1 = 5, x_lat = 37.197934, y_long = -80.580482, z_alt = 10),
				Waypoint(frame = 3, command = 16, is_current = 0, autocontinue = True, param1 = 5, x_lat = 37.197955, y_long = -80.580420, z_alt = 10),
				Waypoint(frame = 3, command = 16, is_current = 0, autocontinue = True, param1 = 5, x_lat = 37.197764, y_long = -80.580429, z_alt = 10),
				Waypoint(frame = 3, command = 16, is_current = 0, autocontinue = True, param1 = 5, x_lat = 37.197727, y_long = -80.580394, z_alt = 10),
				Waypoint(frame = 3, command = 16, is_current = 0, autocontinue = True, param1 = 5, x_lat = 37.197757, y_long = -80.580234, z_alt = 10),
				Waypoint(frame = 3, command = 16, is_current = 0, autocontinue = True, param1 = 5, x_lat = 37.197560, y_long = -80.580247, z_alt = 10),
				Waypoint(frame = 3, command = 16, is_current = 0, autocontinue = True, param1 = 5, x_lat = 37.197433, y_long = -80.580112, z_alt = 10),
				Waypoint(frame = 3, command = 16, is_current = 0, autocontinue = True, param1 = 5, x_lat = 37.197321, y_long = -80.580108, z_alt = 10),
				Waypoint(frame = 3, command = 16, is_current = 0, autocontinue = True, param1 = 5, x_lat = 37.197209, y_long = -80.579865, z_alt = 10),
				Waypoint(frame = 3, command = 16, is_current = 0, autocontinue = True, param1 = 5, x_lat = 37.197312, y_long = -80.579917, z_alt = 10),
				Waypoint(frame = 3, command = 16, is_current = 0, autocontinue = True, param1 = 5, x_lat = 37.197450, y_long = -80.579943, z_alt = 10),
				Waypoint(frame = 3, command = 16, is_current = 0, autocontinue = True, param1 = 5, x_lat = 37.197469, y_long = -80.579711, z_alt = 10),
				Waypoint(frame = 3, command = 16, is_current = 0, autocontinue = True, param1 = 5, x_lat = 37.197731, y_long = -80.579591, z_alt = 10),
				Waypoint(frame = 3, command = 16, is_current = 0, autocontinue = True, param1 = 5, x_lat = 37.197791, y_long = -80.579995, z_alt = 10),
				Waypoint(frame = 3, command = 16, is_current = 0, autocontinue = True, param1 = 5, x_lat = 37.197885, y_long = -80.579991, z_alt = 10)
			]
			pushingWaypoints(waypoints)
			switch_modes(216, "auto", 5)
			break
	finishWaypoints(37.197885, -80.579991) # Checks if waypoints are finished
	clear_pull() # Logistic house keeping
	waiting_ugv(37.197885, -80.579991, 0) # Checks if ugv is there yet
	swtich_modes(0, "guided", 1)
	swtich_modes(0, "auto", 5)

	while True:
		rospy.sleep(2)
		print("Waiting for UAV to be close to next takeoff point")
		if abs(latitude-37.197961)<tolerance and abs(longitude-(-80.579831))<tolerance:
			swtich_modes(0, "stabalize", 5)
			armingCall()
			switch_modes(216, "guided", 5)
			takeoff_call(37.197961, -80.579831, 10)
			waypoints = [
				Waypoint(frame = 3, command = 16, is_current = 1, autocontinue = True, param1 = 5, x_lat = 37.197961, y_long = -80.579831, z_alt = 10),
				Waypoint(frame = 3, command = 16, is_current = 0, autocontinue = True, param1 = 5, x_lat = 37.197961, y_long = -80.579831, z_alt = 10),
				Waypoint(frame = 3, command = 16, is_current = 0, autocontinue = True, param1 = 5, x_lat = 37.198007, y_long = -80.579748, z_alt = 10),
				Waypoint(frame = 3, command = 16, is_current = 0, autocontinue = True, param1 = 5, x_lat = 37.198175, y_long = -80.579745, z_alt = 10),
				Waypoint(frame = 3, command = 16, is_current = 0, autocontinue = True, param1 = 5, x_lat = 37.198114, y_long = -80.580038, z_alt = 10),
				Waypoint(frame = 3, command = 16, is_current = 0, autocontinue = True, param1 = 5, x_lat = 37.198142, y_long = -80.580144, z_alt = 10),
				Waypoint(frame = 3, command = 16, is_current = 0, autocontinue = True, param1 = 5, x_lat = 37.198227, y_long = -80.580195, z_alt = 10),
				Waypoint(frame = 3, command = 16, is_current = 0, autocontinue = True, param1 = 5, x_lat = 37.198308, y_long = -80.580148, z_alt = 10),
				Waypoint(frame = 3, command = 16, is_current = 0, autocontinue = True, param1 = 5, x_lat = 37.198386, y_long = -80.580173, z_alt = 10),
				Waypoint(frame = 3, command = 16, is_current = 0, autocontinue = True, param1 = 5, x_lat = 37.198598, y_long = -80.579973, z_alt = 10),
				Waypoint(frame = 3, command = 16, is_current = 0, autocontinue = True, param1 = 5, x_lat = 37.198703, y_long = -80.579827, z_alt = 10),
				Waypoint(frame = 3, command = 16, is_current = 0, autocontinue = True, param1 = 5, x_lat = 37.198409, y_long = -80.579726, z_alt = 10),
				Waypoint(frame = 3, command = 16, is_current = 0, autocontinue = True, param1 = 5, x_lat = 37.198327, y_long = -80.579470, z_alt = 10),
				Waypoint(frame = 3, command = 16, is_current = 0, autocontinue = True, param1 = 5, x_lat = 37.198286, y_long = -80.579300, z_alt = 10),
				Waypoint(frame = 3, command = 16, is_current = 0, autocontinue = True, param1 = 5, x_lat = 37.198151, y_long = -80.579481, z_alt = 10),
				Waypoint(frame = 3, command = 16, is_current = 0, autocontinue = True, param1 = 5, x_lat = 37.198013, y_long = -80.579360, z_alt = 10),
				Waypoint(frame = 3, command = 16, is_current = 0, autocontinue = True, param1 = 5, x_lat = 37.198018, y_long = -80.579034, z_alt = 10)
			]
			pushingWaypoints(waypoints)
			switch_modes(216, "auto", 5)
			break
	finishWaypoints(37.198018, -80.579034) # Checks if waypoints are finished
	clear_pull() # Logistic house keeping
	waiting_ugv(37.198018, -80.579034, 0) # Checks if ugv is there yet
	swtich_modes(0, "guided", 1)
	swtich_modes(0, "auto", 5)

	while True:
		rospy.sleep(2)
		print("Waiting for UAV to be close to next takeoff point")
		if abs(latitude-37.198388)<tolerance and abs(longitude-(-80.578879))<tolerance:
			swtich_modes(0, "stabalize", 5)
			armingCall()
			switch_modes(216, "guided", 5)
			takeoff_call(37.198388, -80.578879, 10)
			waypoints = [
				Waypoint(frame = 3, command = 16, is_current = 1, autocontinue = True, param1 = 5, x_lat = 37.198388, y_long = -80.578879, z_alt = 10),
				Waypoint(frame = 3, command = 16, is_current = 0, autocontinue = True, param1 = 5, x_lat = 37.198388, y_long = -80.578879, z_alt = 10),
				Waypoint(frame = 3, command = 16, is_current = 0, autocontinue = True, param1 = 5, x_lat = 37.198584, y_long = -80.578570, z_alt = 10),
				Waypoint(frame = 3, command = 16, is_current = 0, autocontinue = True, param1 = 5, x_lat = 37.198542, y_long = -80.579220, z_alt = 10),
				Waypoint(frame = 3, command = 16, is_current = 0, autocontinue = True, param1 = 5, x_lat = 37.198779, y_long = -80.579046, z_alt = 10),
				Waypoint(frame = 3, command = 16, is_current = 0, autocontinue = True, param1 = 5, x_lat = 37.198985, y_long = -80.579420, z_alt = 10),
				Waypoint(frame = 3, command = 16, is_current = 0, autocontinue = True, param1 = 5, x_lat = 37.199013, y_long = -80.579061, z_alt = 10),
				Waypoint(frame = 3, command = 16, is_current = 0, autocontinue = True, param1 = 5, x_lat = 37.199096, y_long = -80.579034, z_alt = 10),
				Waypoint(frame = 3, command = 16, is_current = 0, autocontinue = True, param1 = 5, x_lat = 37.199096, y_long = -80.578936, z_alt = 10),
				Waypoint(frame = 3, command = 16, is_current = 0, autocontinue = True, param1 = 5, x_lat = 37.199006, y_long = -80.578767, z_alt = 10),
				Waypoint(frame = 3, command = 16, is_current = 0, autocontinue = True, param1 = 5, x_lat = 37.199111, y_long = -80.578874, z_alt = 10)
			]
			pushingWaypoints(waypoints)
			switch_modes(216, "auto", 5)
			break
	finishWaypoints(37.199111, -80.578874) # Checks if waypoints are finished
	clear_pull() # Logistic house keeping
	waiting_ugv(37.199111, -80.578874, 0) # Checks if ugv is there yet
	swtich_modes(0, "guided", 1)
	swtich_modes(0, "auto", 5)
	
	# DONE
	print("EVERYTHING WORKED AS PLANNED!!!")
	rospy.spin()


if __name__ == '__main__':
	main()
