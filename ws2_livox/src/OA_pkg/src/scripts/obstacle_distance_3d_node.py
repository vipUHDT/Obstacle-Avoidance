#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header

class ObstacleDistance3D:
	def __init__(self):

		#connecting to LiDAR data
		rospy.Subscriber("/scan", LaserScan, self.pointcloud_callback)

		#publishing MAVlink messages to '/mavros/obstacle/send'
		self.pub = rospy.Publisher("/mavros/obstacle/send", LaserScan, queue_size=10)

		print("OBSTACLE AVOIDANCE SCRIPT IS READY")
			
	def pointcloud_callback(self, msg):
		self.pub.publish(msg)
		print("OBSTACLE_DISTANCE array message sent.")		

if __name__ == '__main__':
	rospy.init_node('obstacle_distance_3d_node')
	obstacle_distance_node = ObstacleDistance3D()
	rospy.spin()
