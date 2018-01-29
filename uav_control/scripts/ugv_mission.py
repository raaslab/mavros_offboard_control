#!/usr/bin/env python

import rospy
import mavros
import sensor_msgs
import yaml
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix

uav_ready = "0"
uav_disarmed = 0.0

def disarm_callback(data):
	global uav_disarmed
	uav_disarmed = data.data

def ready_callback(data):
	# rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
	global uav_ready
	uav_ready = data.data

def waitingForUAV():
	while True:
		if uav_ready == "1": # TODO: check format # check uav_ready
			while True:
				if uav_disarmed == 1: # TODO: check if this is the right format # waiting for uav_disarmed/uav landed
					return
		else: # if uav not ready then wait 5 seconds
			rospy.sleep(5)

def goto_Points(points):
	# TODO: add goto waypoint

def finished_Waypoints():
	# TODO: check if you have reached last waypoint
	while True:
		if # TODO: ugv Finished:
			ugv_ReadyBit.publish("0")
			return
		else:
			rospy.sleep(5)

def sequenceOfEvents(points):
	ugv_ReadyBit.publish("1")
	waitingForUAV() # wait for UAV to land
	goto_Points(points) # TODO: goto next two waypoints
	


def main():
	rospy.init_node('ugv_node')
	ugv_ReadyBit = rospy.Publisher("/mavros/ugv/ready", String, queue_size=10) # Flag topic
	rospy.Subscriber("/mavros/uav/ready", String, ready_callback)
	rospy.Subscriber("/mavros/disarmbit", , disarm_callback) # TODO: subscribe to disarm bit

	goto_Points(waypoints) # TODO: add goto waypoint
	# once you reach point
	sequenceOfEvents(waypoints) # TODO: repeat





if __name__ == '__main__':
	main()
