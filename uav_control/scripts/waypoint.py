#!/usr/bin/env python

import rospy
import mavros
#from mavros.msg import *
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from std_msgs.msg import String

def waypoint_callback(data):
	print("\nwaypoint_callback")
	rospy.loginfo("Got waypoint: %s", data)

def main():
	rospy.init_node('wayPoint')
	rospy.Subscriber("/mavros/mission/waypoints", WaypointList, waypoint_callback)
	
	#Clearing waypoints
	print("\nClearing")
	rospy.wait_for_service("/mavros/mission/clear")
	print("Clearing Waypoints!!!")
	waypoint_clear = rospy.ServiceProxy("/mavros/mission/clear", WaypointClear)
	resp = waypoint_clear()
	print(resp)
	rospy.sleep(5)

	#Arming
	print("\nArming")
	rospy.wait_for_service("/mavros/cmd/arming")
	print("Arming UAV!!!")
	uav_arm = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
	resp = uav_arm(1)
	print(resp)
	#rospy.sleep(5)
	
	#Takeoff
	#print("\nTakeoff")
	#rospy.wait_for_service("/mavros/cmd/takeoff")
	#print("Takeoff UAV!!!")
	#uav_takeoff = rospy.ServiceProxy("/mavros/cmd/takeoff", CommandTOL)
	#resp = uav_takeoff()
	#print(resp)
	#rospy.sleep(5)
	
	#Sending waypoints_push
	print("\nPushing")
	print("Waiting for MAVROS service...")
	rospy.wait_for_service("/mavros/mission/push")
	
	waypoints = [
		Waypoint(frame = 3, command = 16, is_current = True,
				 x_lat = 47.3978422, y_long = 8.5455939, z_alt = 488.02),
	]
	
	waypoint_push = rospy.ServiceProxy("/mavros/mission/push", WaypointPush)
	resp = waypoint_push(waypoints)
	print(resp)
	rospy.sleep(5)


	#Call waypoints_pull
	print("\nPulling")
	rospy.wait_for_service("/mavros/mission/pull")
	print("Calling Waypoint_pull Service")
	waypoint_pull = rospy.ServiceProxy("/mavros/mission/pull", WaypointPull)
	resp = waypoint_pull()
	print(resp)
	rospy.sleep(5)
	
	print("\nbefore rospy.spin()")
	rospy.spin()
	print("\nafter rospy.spin()")
	


if __name__ == '__main__':
	main()

# def  (data):
    # rospy.loginfo("Got waypoints: %s", data)

# def main():
    # rospy.init_node('waypoint')
    # rospy.Subscriber('/mavros/mission/waypoints', WaypointList, handle_waypoints)

    # # Send a waypoint
    # rospy.loginfo("Waiting for MAVROS service...")
    # rospy.wait_for_service('/mavros/mission/push')

    # waypoints = [
            # Waypoint(frame=Waypoint.FRAME_GLOBAL_REL_ALT,
                # command=Waypoint.NAV_WAYPOINT,
                # is_current=True,
                # x_lat=44.57, y_long=-123.27, z_alt=3.0),
            # Waypoint(frame=Waypoint.FRAME_GLOBAL_REL_ALT,
                # command=Waypoint.NAV_WAYPOINT,
                # is_current=True,
                # x_lat=44.58, y_long=-123.27, z_alt=6.0)
        # ]
    # waypoint_push = rospy.ServiceProxy('/mavros/mission/push', WaypointPush)

    # resp = waypoint_push(waypoints)
    # rospy.loginfo(resp)

    # # Call the WaypointPull service
    # rospy.wait_for_service('/mavros/mission/pull')

    # rospy.loginfo("Calling WaypointPull service")
    # waypoint_pull = rospy.ServiceProxy('/mavros/mission/pull', WaypointPull)

    # resp = waypoint_pull()
    # rospy.loginfo(resp)

    # rospy.spin()
