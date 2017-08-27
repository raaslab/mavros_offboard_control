#!/usr/bin/env python

import rospy
import mavros
#from mavros.msg import *
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from std_msgs.msg import String

#def talker():
#    pub = rospy.Publisher('chatter', String, queue_size=10)
#    rospy.init_node('talker', anonymous=True)
#    rate = rospy.Rate(10) # 10hz
#    while not rospy.is_shutdown():
#        hello_str = "hello world %s" % rospy.get_time()
#        rospy.loginfo(hello_str)
#        pub.publish(hello_str)
#        rate.sleep()
#
#if __name__ == '__main__':
#    try:
#        talker()
#    except rospy.ROSInterruptException:
#        pass

def logWayPoint(data):
	rospy.loginfo("Got waypoint: %s", data)

def callback():
	rospy.loginfo("Got waypoints: %s", data)

	
def main():
	rospy.init_node('wayPoint')
	rospy.Subscriber("/mavros/mission/waypoints", WaypointList, callback)
	rospy.loginfo("Waiting for MAVROS service...")
	rospy.wait_for_service("/mavros/mission/push")
	
	waypoints = [
		Waypoint(frame = 3, command = 24, is_current = True,
				 x_lat = 44.57, y_long = -123.27, z_alt = 3.0),
		Waypoint(frame = 3, command = 16, is_current = True,
				 x_lat = 44.57, y_long = -123.27, z_alt = 3.0)
	]
	
	waypoint_push = rospy.ServiceProxy("/mavros/mission/push", WaypointPush)
	resp = waypoint_push(waypoints)
	rospy.loginfo(resp)
	
	
#		wayPointInfo = rospy.Subscriber(mavros.get_topic('mission', 'waypoints'))
#		rospy.loginfo(wayPointInfo)
#		rate.sleep()
#	rospy.Subscriber('/mavros/mission/waypoints ', wayPointList, logWayPoint)
#	
#	# pull waypoints
#	rospy.loginfo("Waiting for Mavros service PULL...")
#	rospy.wait_for_service('/mavros/mission/pull', timeout=None)
#	rospy.ServiceProxy('/mavros/mission/pull', wayPointPull)
#	rospy.loginfo("Pulled wayPoints %s", wayPointPull)
#	
#	# push waypoints
#	rospy.loginfo("Waiting for Mavros service PUSH...")
#	rospy.wait_for_service('/mavros/mission/push', timeout=None)
#	
#	waypoints = [
#		Waypoint(frame=Waypoint.FRAME_GLOBAL_REL_ALT,
#			command=Waypoint.NAV_WAYPOINT,
#			is_current=True,
#			x_lat=44.57, y_long=-123.27, z_alt=3.0)
#		]
#	rospy.ServiceProxy('/mavros/mission/push', wayPointPush)
#	rospy.loginfo("Pushed wayPoints %s", wayPointPush(waypoints))
#	

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
 




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


# if __name__ == '__main__':
    # main()
