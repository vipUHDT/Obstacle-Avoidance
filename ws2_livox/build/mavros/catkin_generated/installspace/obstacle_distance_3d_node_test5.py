#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header

class ObstacleDistance3D:
	def __init__(self):

		#connecting to LiDAR data
		rospy.Subscriber("/scan", LaserScan, self.pointcloud_callback)

		#publishing MAVlink messages to '/mavros/obstacle/send'
		self.pub = rospy.Publisher('/mavros/obstacle/send', LaserScan, queue_size=10)

		print("OBSTACLE AVOIDANCE SCRIPT IS READY")
			
	def pointcloud_callback(self, scan_msg):
		from sensor_msgs.msg import LaserScan
		cleaned = LaserScan()
		cleaned.header.stamp = rospy.Time.now()
		cleaned.header.frame_id = scan_msg.header.frame_id
		cleaned.angle_min = scan_msg.angle_min
		cleaned.angle_max = scan_msg.angle_max
		cleaned.angle_increment = scan_msg.angle_increment
		cleaned.time_increment = scan_msg.time_increment
		cleaned.scan_time = scan_msg.scan_time
		cleaned.range_min = scan_msg.range_min
		cleaned.range_max = scan_msg.range_max

		cleaned.ranges = [
        r if (r > 0 and r != float('inf')) else scan_msg.range_max
        for r in scan_msg.ranges
    	]
		cleaned.intensities = []

		self.pub.publish(cleaned)
		
		print("OBSTACLE_DISTANCE array message sent.")		

if __name__ == '__main__':
	rospy.init_node('obstacle_distance_3d_node')
	obstacle_distance_node = ObstacleDistance3D()
	rospy.spin()
