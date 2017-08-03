#!/usr/bin/python

import rospy
import mavros
from mavros.utils import *
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, StreamRate, StreamRateRequest, CommandBool, CommandTOL
from geometry_msgs.msg import *
import time



def state_cb(state):
    print state

def flight():

	
	x_pos = [5,-5,-5,5]
	y_pos = [5,5,-5,-5]
	z_pos = [5,5,5,5]
	'''
	# initialize the subscriber node
	rospy.init_node('flight', anonymous=True)
	# subcribe to the mavros State
	#rospy.Subscriber("mavros/state",State,state_cb)
	state = rospy.wait_for_message("mavros/state",State)
	#state_cb(state)
	local_pos_pub = rospy.Publisher("mavros/setpoint_position/local",PoseStamped,queue_size=10)
	rospy.wait_for_service("mavros/cmd/arming");
	print " Arming service available"
	try:
		arming_client = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e
		
	rospy.wait_for_service("mavros/set_mode");
	
	print " SetMode service available"
	
	try:
		set_mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e 
		
	rate = rospy.Rate(20.0)
	while not rospy.is_shutdown() and not state.connected:
		state = rospy.wait_for_message("mavros/state",State)
		rate.sleep()
	'''	
	# initialize the subscriber node
	rospy.init_node('flight', anonymous=True)
	# subcribe to the mavros State
	state = rospy.wait_for_message("mavros/state",State)

	#set the publisher for sending the goals
	localPosPub = rospy.Publisher("mavros/setpoint_position/local",PoseStamped,queue_size=10)


	rospy.wait_for_service("mavros/cmd/arming");
	print " Arming service available"
	try:
		armingClient = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

	rospy.wait_for_service("mavros/set_mode");
	print " SetMode service available"
	try:
		setModeClient = rospy.ServiceProxy('mavros/set_mode', SetMode)
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

	rospy.wait_for_service("mavros/cmd/takeoff");
	print " Takeoff service available"
	try:
		takeoffClient = rospy.ServiceProxy('mavros/cmd/takeoff', CommandTOL)
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e



	rate = rospy.Rate(20.0)
	while not rospy.is_shutdown() and not state.connected:
		state = rospy.wait_for_message("mavros/state",State)
		rate.sleep()

	pose = PoseStamped()
	'''
	pose.pose.position.x = 2;
	pose.pose.position.y = 2;
	pose.pose.position.z = 5;
	
	for i in range(0,100):
 		local_pos_pub.publish(pose);
 		print 'Published setpoint'
		rate.sleep()
		
		
	set_mode_client(0,'OFFBOARD')
	print "******** OFFBOARD ENABLED *****"
	arming_client(True)
	print "****** ARMED ******"
	
	
	set_mode_client(0,'POSCTL')
	'''
	last_request =  rospy.get_rostime()
	i = 0
	while not rospy.is_shutdown():
		'''
		state = rospy.wait_for_message("mavros/state",State) 
		if state.mode != 'OFFBOARD' :#and ((rospy.get_rostime() - last_request) > rospy.Duration(5.0)):
			set_mode_client(0,'OFFBOARD')
			print "OFFBOARD ENABLED"	
			last_request =  rospy.get_rostime()
		
		if not state.armed: #and ((rospy.get_rostime() - last_request) > rospy.Duration(5.0)):
			arming_client(True)
        	print "ARMED"
    		last_request =  rospy.get_rostime()
    	'''
    	
		state = rospy.wait_for_message("mavros/state",State)
		if state.mode != 'OFFBOARD':
			setModeClient(0,'OFFBOARD')
			print "OFFBOARD ENABLED"

		if not state.armed:
				armingClient(True)
				print "ARMED"
            
    	pose.pose.position.x = x_pos[i] 
    	pose.pose.position.y = y_pos[i]
    	pose.pose.position.z = z_pos[i]
    	
    	#for i in range(0,10):
    	local_pos_pub.publish(pose);
    	#	time.sleep(0.1)
    		
    	curr_pose = rospy.wait_for_message("/mavros/local_position/pose" ,PoseStamped)
        if abs(curr_pose.pose.position.x - x_pos[i]) < 0.2 and  abs(curr_pose.pose.position.y - y_pos[i]) < 0.2:
        	i = i + 1
        	if i == 4:
        		i = 0
    	#time.sleep(0.2)
    	#rate.sleep()
		
	'''
	rate = rospy.Rate(100.0)
	while not rospy.is_shutdown():		
		print "Waiting for message" 
		state = rospy.wait_for_message("mavros/state",State) 

		mode = state.mode
		armed = state.armed
		
		print mode
		print armed
			
        pose.pose.position.x = 2
        pose.pose.position.y = 2
        pose.pose.position.z = 5
        
        if mode != 'OFFBOARD':
			set_mode_client(0,'OFFBOARD')
			print "OFFBOARD ENABLED"
       	
       	if not armed:
        	arming_client(True)
        	print "ARMED"
        	
        for i in range(0,10):
       		local_pos_pub.publish(pose);
       		time.sleep(0.1)
    
        print i
        
        #pose.pose.position.x = x_pos[i]
        #pose.pose.position.y = y_pos[i]
        #pose.pose.position.z = z_pos[i]
        
        curr_pose = rospy.wait_for_message("/mavros/local_position/pose" ,PoseStamped)
        if abs(curr_pose.pose.position.x - x_pos[i]) < 0.2 and  abs(curr_pose.pose.position.y - y_pos[i]) < 0.2:
        	i = i + 1
        	if i == 4:
        		i = 0
		
	state = rospy.wait_for_message("mavros/state",State)
	
	if state.armed:
		arming_client(False)
		print "DISARMED"
    '''        
		#rate.sleep()
if __name__ == '__main__':
    try:
        flight()
    except rospy.ROSInterruptException:
        pass
        #curr_pose = rospy.wait_for_message("/mavros/local_position/pose" ,PoseStamped)
        #pose.pose.position.x = curr_pose.pose.position.x
        #pose.pose.position.y = curr_pose.pose.position.y
        #pose.pose.position.z = curr_pose.pose.position.z
        #local_pos_pub.publish(pose)