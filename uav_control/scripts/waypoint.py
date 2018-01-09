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

def waiting_ugv(lat, long, alt):
	print("\n----------waiting_ugv----------")
	while True:
		# TODO: add listener to the UGV flag here
		# checker = UGV publisher
		checker = "ready"
		if checker == "ready"
			waypoints = [Waypoint(frame = 3, command = 21, is_current = True, autocontinue = True, param1 = 5, x_lat = lat, y_long = long, z_alt = alt)]
			waypoint_push = rospy.ServiceProxy("/mavros/mission/push", WaypointPush)
			resp = waypoint_push(waypoints)
			#print(resp)
			rospy.sleep(5)
			return

def clear_pull():
	print("\n----------clear_pull----------")
	#Clearing waypoints
	#print("\n----------CLEARING----------")
	rospy.wait_for_service("/mavros/mission/clear")
	#print("Clearing Waypoints!!!")
	waypoint_clear = rospy.ServiceProxy("/mavros/mission/clear", WaypointClear)
	resp = waypoint_clear()
	#print(resp)
	rospy.sleep(5)

	#Call waypoints_pull
	#print("\n----------PULLING----------")
	rospy.wait_for_service("/mavros/mission/pull")
	#print("Calling Waypoint_pull Service")
	waypoint_pull = rospy.ServiceProxy("/mavros/mission/pull", WaypointPull)
	resp = waypoint_pull()
	#print(resp)
	rospy.sleep(5)
	return

def finishwaypoints():
	print("\n----------finishwaypoints----------")
	while True:						#waits for last_waypoint in previous WaypointList to be visited
		rospy.sleep(2)
		#print("WAITING for last_waypoint to be true")
		if last_waypoint == True:			#if last_waypoint is in the process of being visited
			while True:
				rospy.sleep(2)
				#print("WAITING for last_waypoint to be false")
				if last_waypoint == False:	#if last_waypoint has been visited (due to previous constraint)
					break
			break
	return


def main():
	rospy.init_node('wayPoint')
	rospy.Subscriber("/mavros/mission/waypoints", WaypointList, waypoint_callback)
	rospy.Subscriber("/mavros/global_position/raw/fix", NavSatFix, globalPosition_callback)
	#Flag topic
	readyBit = rospy.Publisher("/mavros/ugv/ready", String, queue_size=10)
	ready_str = "ready"
	notReady_str = "notReady"
    
	clear_pull()
		
	#Sending waypoints_push
	print("\n----------PUSHING----------")
	#print("Waiting for MAVROS service...")
	rospy.wait_for_service("/mavros/mission/push")
	
	waypoints = [
		Waypoint(frame = 3, command = 22, is_current = True, autocontinue = True, param1 = 5, x_lat = 37.1977394, y_long = -80.5794510, z_alt = 10),
		Waypoint(frame = 3, command = 16, is_current = False, autocontinue = True, param1 = 5, x_lat = 37.1976407, y_long = -80.5795481, z_alt = 5),
		Waypoint(frame = 3, command = 16, is_current = False, autocontinue = True, param1 = 5, x_lat = 37.1975393, y_long = -80.5796954, z_alt = 5)
	]
	
	waypoint_push = rospy.ServiceProxy("/mavros/mission/push", WaypointPush)
	resp = waypoint_push(waypoints)
	#print(resp)
	rospy.sleep(5)
	
	finishwaypoints() #checks if waypoints are finished
	
	clear_pull() #logistic house keeping
	
	waiting_ugv(37.1975393, -80.5796954, 0) #checks if ugv is there yet

	while True:
		rospy.sleep(2)
		#print("WAITING for us to be within 1 meter of the next takeoff point")
		#print(" lat " + repr(latitude) + " long " + repr(longitude) + " alt " + repr(altitude))
		if (latitude-37.1973420)<0.0001 and (longitude-(-80.5798929))<0.0001 and (altitude-529)<2:
			rospy.wait_for_service("/mavros/mission/push")
			resp = waypoint_push(waypoints)
			waypoints = [
				Waypoint(frame = 3, command = 22, is_current = True, autocontinue = True, param1 = 5, x_lat = 37.1973420, y_long = -80.5798929, z_alt = 10),
				Waypoint(frame = 3, command = 16, is_current = False, autocontinue = True, param1 = 5, x_lat = 37.1972726, y_long = -80.5799733, z_alt = 5),
				Waypoint(frame = 3, command = 16, is_current = False, autocontinue = True, param1 = 5, x_lat = 37.1971499, y_long = -80.5801173, z_alt = 5)
			]
			resp = waypoint_push(waypoints)
			#print(resp)
			rospy.sleep(5)
			break

	finishwaypoints() #checks if waypoints are finished
	
	waiting_ugv(37.1971499, -80.5801173, 0) #checks if ugv is there yet

	print("EVERYTHING WORKED AS PLANNED!!!")
	rospy.spin()


if __name__ == '__main__':
	main()