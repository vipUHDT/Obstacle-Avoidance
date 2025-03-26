#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import math
import time
import multiprocessing
import threading
from threading import Thread
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
from pymavlink import mavutil
from haversine import haversine, Unit

class ObstacleDistance3D:

	def __init__(self):
		
		#PARAMTERS
		self.ALTITUDE = 22.8 # meters
		self.alt_AD = 26
		self.alt_IP = 27
		self.WAYPOINT_RADIUS = 5 # feet
		self.PAYLOAD_RADIUS = 5 # feet
		self.SEARCH_AREA_RADIUS = 5 # feet
		
		self.WAYPOINT_SPEED = 0.5 # m/s
		
		self.SEARCH_SPEED = 15 # m/s
		self.DELIVER_SPEED = 20 # m/s
		#connecting to UAS with dronekit
		print("Connecting to UAS")
		#self.connection_string = 'udp:127.0.0.1:14551' #Software in the loop
		self.connection_string = "/dev/ttyACM0" #usb to micro usb
		# self.SK = ServoKit( channels = 16 


		#Connect to DroneKit
		self.connect_to_dronekit()

		#Obstacle Avoidance Parameters
		self.UAS_dk.parameters['PRX1_TYPE'] = 2
		self.UAS_dk._parameters['FENCE_ENABLE'] = 0

		self.UAS_dk._parameters['AVOID_ENABLE'] = 1

		self.UAS_dk._parameters['AVOID_BEHAVE'] = 1
		# 0: to slide 1: to stop completely

		#self.UAS_dk._parameters['AVOID_BACKUP_SPD'] = 1
		# speed to back if in margin zone (m/s) from 0 - 2

		#self.UAS_dk._parameters['AVOID_ALT_MIN'] = 0
		# alt to ignore obstacles for autonomous launch

		#self.UAS_dk._parameters['AVOID_BACKUP_DZ'] = 0
		# increase if going back and forth in front obstacle

		#self.UAS_dk._parameters['AVOID_BACKZ_SPD'] = 0
		# vertical speed to avoid


		#self.UAS_dk._parameters['OA_TYPE'] = 1 
		# 1: Bendy Ruler 3: Dijkstra's

		self.UAS_dk._parameters['OA_BR_TYPE'] = 1
		# 1: Horizontal search 2: Vertical search

		self.UAS_dk._parameters['OA_BR_LOOKAHEAD'] = 40 
		# meters to look ahead usb to micro usb

		self.UAS_dk._parameters['OA_LOOKAHEAD'] = 40 
		# meters to look ahead Software in the loop

		self.UAS_dk._parameters['OA_MARGIN_MAX'] = 7 
		# meters away from obstacle

		self.UAS_dk._parameters['OA_DB_EXPIRE'] = 30
		# seconds obstacle stay in database

		#self.UAS_dk._parameters['OA_DB_OUTPUT'] = 3
		# 3 show all obstacles on map

		self.UAS_dk._parameters['GUID_OPTIONS'] = 64
		# Uses bits (6th bit for OA in guided mode)


		
		#self.UAS_dk.reboot()
		print("OA_TYPE set to: ", self.UAS_dk.parameters['OA_TYPE'])
		#print("OA_BR_LOOKAHEAD set to: ", self.UAS_dk.parameters['OA_BR_LOOKAHEAD']) # usb to micro usb
		#print("OA_BR_LOOKAHEAD set to: ", self.UAS_dk.parameters['OA_LOOKAHEAD']) #Software in the loop
		print("OA_MARGIN_MAX set to: ", self.UAS_dk.parameters['OA_MARGIN_MAX'])
		print("OA_BR_TYPE set to: ", self.UAS_dk.parameters['OA_BR_TYPE'])
		print("GUID_OPTIONS set to: ", self.UAS_dk.parameters['GUID_OPTIONS'])


		#connecting to LiDAR data
		rospy.Subscriber("/livox/lidar", PointCloud2, self.pointcloud_callback)
		
		# writing file variable
		self.attitude_time = []
		self.deliver_payload_time = []
		self.geotag_time = []
		self.haversine_time = []
		self.search_area_waypoint_time = []
		self.subprocess_execute_time = []
		self.trigger_camera_time = []
		self.waypoint_lap_time = []
		self.dk_waypoint_lap_time = []
		self.payload_delivery_time = []
		
		#declaring initial variable
		self.pitch = 0.0
		self.roll = 0.0
		self.yaw = 0.0
		self.lat = 0.0
		self.lon = 0.0
		self.alt = 0.0
		self.image_number = 1
		self.drone_sensory = [self.pitch, self.roll, self.yaw, self.lat, self.lon, self.alt]
		self.currWP_index = 0
		self.lap = 1
		self.payload = 1
		self.filename = f"image"

		self.waypoint_lap_latitude = [
			21.4001570,
			21.4004305
		]
		self.waypoint_lap_longitude = [
			-157.7645583,
			-157.7639428
		]

		self.waypoint_lap_alt = [
			22.8,
            22.8
		]
		
		print("AUTONOMOUS SCRIPT IS READY")

		while (self.IS_ARMED() != True):
			print("waiting to be armed")
			print(self.UAS_dk.armed)
			time.sleep(1)
		print("UAS IS NOW ARMED")

		while (self.IS_GUIDED() != True):
			print("waiting to be in GUIDED mode")
			print(self.UAS_dk.mode)
			time.sleep(1)

		

		print("UAS IS NOW IN GUIDED MODE")
		print("!------------------ MISSION STARTING ----------------------!")
		
	def connect_to_dronekit( self ):
		print( "Connecting to DroneKit" )

		self.UAS_dk = connect( self.connection_string, baud = 57600, wait_ready = True, heartbeat_timeout = 180, source_system=1)

		print( "Connected to DroneKit ")
	
	def reboot(self):
		print("Rebooting autopilot...")
		msg = self.UAS_dk.message_factory.command_long_encode(
			1,
			0,
			mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
			0,
			1,
			0, 0, 0, 0, 0, 0
		)
		self.UAS_dk.send_mavlink(msg)

	def RTL_stat( self ):
		"""
		Check if the UAS is in "Return to Launch" mode.

		Returns:
			bool: True if the UAS is in RTL mode, False otherwise.
		"""
		#self.UAS_dk = connect(self.connection_string, baud=57600, wait_ready=True)

		if self.UAS_dk.mode == "RTL":
			return True
		return False
			
	def haversine(self, lat1, lon1):
		"""
		Use the Haversine formula to calculate the distance between two coordinates.

		Args:
			lon1 (float): Longitude of the first coordinate.
			lat1 (float): Latitude of the first coordinate.

		Returns:
			float: The distance between the two coordinates in meters.
		"""
		current_location = self.UAS_dk.location.global_relative_frame
		distance = haversine((current_location.lat,current_location.lon),(lat1,lon1), unit = 'ft')
		
		return distance

	def velocity_loop(self):
		while not rospy.is_shutdown():
			msg = self.UAS_dk.message_factory.set_position_target_local_ned_encode(
				0, 0, 0, mavutil.mavlink.MAV_FRAME_LOCAL_NED,
				0b0000111111000111,
				0, 0, 0,
				self.WAYPOINT_SPEED, 0, 0,
				0, 0, 0,
				0, 0
			)
			self.UAS_dk.send_mavlink(msg)
			time.sleep(0.5)

			if self.currWP_index == 2:
				self.WAYPOINT_SPEED = 0


		

	def waypoint_reached (self, latitude_deg, longitude_deg, radius ):
		"""
		Check if the UAS has reached a specified waypoint.

		Args:
			latitude_deg (float): The latitude coordinate of the waypoint.
			longitude_deg (float): The longitude coordinate of the waypoint.

		Returns:
			bool: True if the waypoint is reached, False otherwise.
		"""
		#distance between 2 points retuirn value in feet    
		distance = self.haversine(latitude_deg,longitude_deg )
		#checking is UAS reached within 15 feet in diameter of the desired coordinate desitination

		while(distance >= radius):
			if(self.RTL_stat() == True):
				print("IN RTL MODE")

				while (self.RTL_stat() == True):
					print("IN RTL MODE")
					time.sleep(.5)
					pass
				self.UAS_dk.simple_goto(LocationGlobalRelative(latitude_deg,longitude_deg,self.ALTITUDE))
				self.waypoint_reached( latitude_deg, longitude_deg, radius )
				break
			#distance between 2 points retuirn value in feet    
			distance = self.haversine(latitude_deg, longitude_deg)            
			print(f"Distance to waypoint: {distance}")
			time.sleep(.5)
		print("REACHED WAYPOINT") 
		self.currWP_index += 1
		return True     
		return distance
	
	def IS_ARMED(self):
		"""
		Check if the UAS is "ARMED" mode.

		Returns:
			bool: True if the UAS is ARMED, False otherwise.
		"""  

		
		while not self.UAS_dk.armed:
			print( "Waiting to be ARMED" )
			time.sleep( 1 )

		print( "UAS is ARMED" )
		return True

	def IS_AUTO(self):
		"""
		Check if the UAS is in "AUTO" mode.

		Returns:
			bool: True if the UAS is in AUTO, False otherwise.
		"""    
		
		
		while self.UAS_dk.mode != "AUTO":
			self.IS_AUTO()
			print( "Waiting to be in AUTO" )
			time.sleep( 1 )

		print( "UAS is in AUTO" )
		return True
	
	def IS_GUIDED(self):
		"""
		Check if the UAS is in "AUTO" mode.

		Returns:
			bool: True if the UAS is in AUTO, False otherwise.
		"""        

		while self.UAS_dk.mode != "GUIDED":
			print( "Waiting to be in GUIDED" )
			time.sleep( 1 )
		
		# rospy.spin()
		print( "UAS is in GUIDED" )
		return True
	
	def dk_waypoint_lap( self ):
	
		self.UAS_dk.mode = VehicleMode("GUIDED") 

		start = time.time()

		for wp in range(len(self.waypoint_lap_latitude)):
			# self.UAS_dk = connect(self.connection_string, baud=57600, wait_ready=True)
			print(LocationGlobalRelative(self.waypoint_lap_latitude[ wp ], self.waypoint_lap_longitude[ wp ],self.ALTITUDE))
			self.UAS_dk.simple_goto(LocationGlobalRelative(self.waypoint_lap_latitude[ wp ], self.waypoint_lap_longitude[ wp ],self.waypoint_lap_alt[ wp ]), groundspeed = self.WAYPOINT_SPEED )
			self.waypoint_reached(self.waypoint_lap_latitude[ wp ], self.waypoint_lap_longitude[ wp ], self.WAYPOINT_RADIUS)
		
		end = time.time()
		
		difference = end - start
		
		self.dk_waypoint_lap_time.append(difference)
		
		self.lap = self.lap + 1
		
		return print(f"DONE WITH LAP {self.lap - 1}")
			

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
	  
	  
	def send_obstacle_distance_3d(self, x, y, z):
		timestamp = int(rospy.get_time() * 1e3) # miliseconds
		frame = mavutil.mavlink.MAV_FRAME_BODY_FRD # FRD possibly not supported
		min_distance = 1
		max_distance = 60
		
		self.mavlink_connection.mav.obstacle_distance_3d_send(
		timestamp,
		0,
		frame,
		65535,
		x,
		y,
		z,
		min_distance,
		max_distance
		)


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
		sensor_type = 0
		increment_f = angle_increment
		angle_offset = 0.0
		frame = mavutil.mavlink.MAV_FRAME_BODY_FRD
	
		# Send OBSTACLE_DISTANCE message
		OAmsg = self.UAS_dk.message_factory.obstacle_distance_encode(
			timestamp,
			sensor_type,
			distances,
			int(angle_increment),
			min_distance_cm,
			max_distance_cm,
			increment_f,
			angle_offset,
			frame
		)
		self.UAS_dk.send_mavlink(OAmsg)
		print("OBSTACLE_DISTANCE array message sent.")


if __name__ == '__main__':
	rospy.init_node('obstacle_distance_3d_node')
	obstacle_distance_node = ObstacleDistance3D()
	
	#p1 = multiprocessing.Process(target=obstacle_distance_node.dk_waypoint_lap)
	velocity_thread = threading.Thread(target=obstacle_distance_node.velocity_loop)
	waypoint_thread = threading.Thread(target=obstacle_distance_node.dk_waypoint_lap)
	time.sleep(5)
	#rospy.spin()
	velocity_thread.start()
	waypoint_thread.start()
	rospy.spin()

	

	

	


