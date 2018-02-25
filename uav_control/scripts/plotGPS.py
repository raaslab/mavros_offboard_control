#!/usr/bin/env python

import rospy
import mavros
import sensor_msgs
import yaml
import utm
import time
import matplotlib.pyplot as plt
import xlsxwriter
from mpl_toolkits.basemap import Basemap
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from std_msgs.msg import String, Int64
from sensor_msgs.msg import NavSatFix

latitude = 37.0
longitude = -80.0


def waypoint_callback(data):
	global last_waypoint
	if len(data.waypoints) !=0:
		last_waypoint = data.waypoints[len_data.waypoints-1].is_current
	return

def globalPosition_callback(data):
	global latitude
	global longitude
	global altitude
	latitude = data.latitude
	longitude = data.longitude
	altitude = data.altitude

def plotData(xData, yData, axis, n):
	# xData = [i for i in xData if i != xDel]
	# yData = [i for i in yData if i != yDel]
	xData = xData[n:]
	yData = yData[n:]
	plt.plot(xData, yData, 'ro')
	if axis == 0:
		return
	else:
		plot.axis(axis)

def main():
	rospy.init_node('gpsPlotter')
	# rospy.Subscriber("/mavros/mission/waypoints", WaypointList, waypoint_callback)
	rospy.Subscriber("/mavros/global_position/raw/fix", NavSatFix, globalPosition_callback)
	rate = rospy.Rate(1)
	xData = [0,0]
	yData = [0,0]
	counter = 0
	totalTime = 460
	resolution = 0.5
	numDelPoints = 20	
	while not rospy.is_shutdown():
		try:
			[x, y, rand, rand1] = utm.from_latlon(latitude, longitude)
			print(counter, "- x: ", x, " latitude: ", latitude, " longitude: ", longitude, " y: ", y)
			# print(len(xData))
			# print(xData[-1])
			if len(xData) > 1:
				if x != xData[-1]:
					xData.append(x)
					yData.append(y)
					# time.sleep(resolution)
					counter = counter+1
					# if counter == totalTime:
					# 	print("break")
					# 	break
		except KeyboardInterrupt:
			print("All done")

	# [xDel, yDel, rand, rand1] = utm.from_latlon(0,0)
	# map = Basemap(llcrnrlon=-85,llcrnrlat=30,urcrnrlon=-70,urcrnrlat=40,
 #             resolution='f', projection='cyl')
	# map.bluemarble()
	# map.drawcoastlines()
	workbook   = xlsxwriter.Workbook('data.xlsx')
	worksheet1 = workbook.add_worksheet()
	worksheet1.write_column(0,0,xData)
	worksheet1.write_column(0,1,yData)

	# plt.show()
	# print(xData)
	plotData(xData, yData, 0, numDelPoints)
	plt.show()
	# time.sleep(1)
	print("Finished")

if __name__ == '__main__':
	main()