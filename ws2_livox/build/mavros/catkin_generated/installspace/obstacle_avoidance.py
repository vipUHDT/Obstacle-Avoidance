#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2, LaserScan
from mavros_msgs.msg import ObstacleDistance
from std_msgs.msg import Header
import sensor_msgs.point_cloud2 as pc2
import math
from pymavlink import mavutil

class ObstacleDistance3D:
	def __init__(self):
		
		#connecting to UAS with dronekit
		#print("Connecting to UAS")
		#self.connection_string = 'udp:127.0.0.1:14551' #Software in the loop
		#self.connection_string = "/dev/ttyACM0" #usb to micro usb
		# self.SK = ServoKit( channels = 16 )

		#connecting to LiDAR data
		rospy.Subscriber("/livox/lidar", PointCloud2, self.pointcloud_callback)

		#publishing MAVlink messages to '/mavros/obstacle/send'
		pub = rospy.Publisher('/mavros/obstacle/send', ObstacleDistance, queue_size=10)

		print("OBSTACLE AVOIDANCE SCRIPT IS READY")
		
		return True
			

	def pointcloud_callback(self, msg):
		# Process the downsampled point cloud
		obstacle_data = []

		# Iterate over points in the downsampled cloud
		for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
			
			x, y, z = point  # Extract 3D coordinates
			#x = abs(x)
			#y = abs(y)
			#z = abs(z)
			#print("extract X: {}, Y: {}, Z: {}".format(x, y, z))
		  

			# Only consider obstacles within a certain range
			distance = (x**2 + y**2 + z**2) ** 0.5
			if 1 < distance < 100: # Change values here
				obstacle_data.append((x, y, z))
			
		self.send_obstacle_distance_array(msg)
		
		#for x, y, z in obstacle_data:
		 #   self.send_obstacle_distance_3d(x, y, z)


	def send_obstacle_distance_array(self, msg):
		"""
		Send the OBSTACLE_DISTANCE message using a 72-element array
		with 5-degree increments, covering 360 degrees around the vehicle.
		"""
		num_readings = 72
		angle_increment = 360 / num_readings  # 5 degrees per element
		max_distance_cm = 3000  # Max distance of 30m in cm
		min_distance_cm = 100   # Min distance of 1m in cm
		distances = [65535] * num_readings  # Initialize with no-obstacle value

		for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
			x, y, z = point
			distance = math.sqrt(x**2 + y**2 + z**2) * 100  # Convert to cm
			if min_distance_cm < distance < max_distance_cm:
				angle = (math.degrees(math.atan2(y, x)) + 360) % 360
				index = int(angle / angle_increment) % num_readings
				distances[index] = min(distances[index], int(distance))

		timestamp = int(rospy.get_time() * 1e6)
		sensor_type = ObstacleDistance.SENSOR_TYPE_LASER
		increment_f = angle_increment
		angle_offset = 0.0
		frame = mavutil.mavlink.MAV_FRAME_BODY_FRD
	
		# Build OBSTACLE_DISTANCE message
		msg = ObstacleDistance()
		msg.header = Header()
		msg.header.stamp = rospy.Time.now()
		msg.sensor_type = sensor_type
		msg.distances = distances
		msg.increment = int(angle_increment)
		msg.min_distance = min_distance_cm
		msg.max_distance = max_distance_cm
		msg.angle_offset = angle_offset
		msg.frame = frame

		self.pub.publish(msg)
		print("OBSTACLE_DISTANCE array message sent.")


if __name__ == '__main__':
	rospy.init_node('obstacle_distance_3d_node')
	obstacle_distance_node = ObstacleDistance3D()
	rospy.spin()

	

	

	


