# ROS Imports
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile, ReliabilityPolicy

from rclpy.action.client import ActionClient
from rclpy.action.client import GoalStatus
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from action_msgs.msg import GoalStatus
from std_msgs.msg import Header, String, Int32, Float32, UInt8, Int16,Bool # Some topics have specific datatypes (POTENTIALLY USELESS!!!)


# Create3 Packages
import irobot_create_msgs
from irobot_create_msgs.action import DriveDistance, Undock, RotateAngle, Dock, NavigateToPosition, AudioNoteSequence
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry  # Import Odometry
from irobot_create_msgs.srv import ResetPose
from irobot_create_msgs.msg import AudioNoteVector, AudioNote, IrIntensityVector, IrOpcode
from builtin_interfaces.msg import Duration

# key_commander stuff
from pynput.keyboard import KeyCode
from key_commander import KeyCommander

# Python Packages
import random, time
from math import pi
from collections import deque
import json
from datetime import datetime


# Threading
from rclpy.executors import MultiThreadedExecutor
from threading import RLock
import threading

# Broker
# import paho.mqtt.client as mqtt
from brokerSender import mqttc

# Node Imports
from dock_status_node import DockStatusMonitorNode
from ir_status_node import IrMonitorNode
from odometry_node import OdomNode

# Globals
start_note = [ AudioNote(frequency=600, max_runtime=Duration(sec=0, nanosec= 50000000))]
end_note =   [ AudioNote(frequency=784, max_runtime=Duration(sec=0, nanosec= 50000000))]
rand_note =  [ AudioNote(frequency=350, max_runtime=Duration(sec=0, nanosec= 150000000))]
ready_notes = [
	AudioNote(frequency=583, max_runtime=Duration(sec=0, nanosec= 50000000)),
	AudioNote(frequency=784, max_runtime=Duration(sec=0, nanosec=100000000))
]
error_notes = [
	AudioNote(frequency=583, max_runtime=Duration(sec=0, nanosec= 50000000)),
	AudioNote(frequency=350, max_runtime=Duration(sec=0, nanosec=100000000))
]

topic = "vandalrobot"


# Initialize and connect to other nodes
rclpy.init()
namespace = 'create3_05AE'
dock_sensor = DockStatusMonitorNode(namespace)
ir_sensor = IrMonitorNode(namespace)
odometry_sensor = OdomNode(namespace)

class Roomba(Node):
	def __init__(self, namespace):
		super().__init__('roomba_node')
		# Establish the node objects within the class for node operations
		self.dock_sensor = dock_sensor
		self.ir_sensor = ir_sensor
		self.odometry_sensor = odometry_sensor

		# Subscriptions: 
		# Split up to compensate for noisy subscriptions
		cb_dockstatus = MutuallyExclusiveCallbackGroup() # Perhaps unneeded since the dock_status_node takes care of the dockstatus
		cb_ir = MutuallyExclusiveCallbackGroup()
		cb_pose = MutuallyExclusiveCallbackGroup()

		self.dock_status_sub_ = self.create_subscription(Bool, f'/{namespace}/check_dock_status', 
												self.dock_status_callback, 10, callback_group=cb_dockstatus)
		# self.ir_opcode_sub_ = self.create_subscription(IrOpcode, f'/{namespace}/ir_opcode_number', 
		# 										self.ir_opcode_callback, qos_profile_sensor_data, callback_group=cb_ir)
		self.ir_opcode_sub_ = self.create_subscription(String, f'/{namespace}/ir_opcode_number', 
														self.ir_opcode_callback, qos_profile_sensor_data, callback_group=cb_ir)

		self.current_pose_sub_ = self.create_subscription(PoseStamped, f'/{namespace}/pose_stamped', 
												self.pose_callback, qos_profile_sensor_data, callback_group=cb_pose)

		# Actions:
		cb_Action = MutuallyExclusiveCallbackGroup()
		cb_chirp  = MutuallyExclusiveCallbackGroup()

		self.dock_ac = ActionClient(self, Dock, f'/{namespace}/dock',
							  				callback_group=cb_Action)
		self.undock_ac = ActionClient(self, Undock, f'/{namespace}/undock',
								 			callback_group=cb_Action)
		self.drive_ac = ActionClient(self, DriveDistance, f'/{namespace}/drive_distance', 
							   				callback_group=cb_Action)
		self.nav_to_pos_ac = ActionClient(self, NavigateToPosition, f'/{namespace}/navigate_to_position', 
											callback_group=cb_Action)
		self.audio_ac = ActionClient(self, AudioNoteSequence, f'/{namespace}/audio_note_sequence', 
											callback_group=cb_chirp)
		self.rotate_ac = ActionClient(self, RotateAngle, f'/{namespace}/rotate_angle', 
											callback_group=cb_Action)
		
		# Services:
		# ResetPose service client and initialize PoseStamped variable for position reset using odometry
		self.reset_pose_srv = self.create_client(ResetPose, f'/{namespace}/reset_pose')
		
		# Ensure service is available
		while not self.reset_pose_srv.wait_for_service(timeout_sec=1.0):
			self.get_logger().info('ResetPose service not available, waiting again...')

		# Variable Initialization
		self.latest_dock_status = True #########!!!!!!!!!!!!! Was None
		self.latest_pose_stamped = None
		self.latest_ir_opcode = None
		self.ir_opcode_history = deque(maxlen=20)  # A deque to store the history of opcodes

	def reportSender(self, label="Undefined", action="Undefined",
					isAtBase1=False, isAtBase2=False, isAtBase3=False,
					isReady=False, isMoving=False, iswithDice=False
					):
		""""
		Used to send a report at the beginning and end of every action.
		Defaults to False.
		"""

		self.record_pose() # This should update the recorded pose whenever the report is sent
		# time.sleep(1)

		data = {
			"messageType": "Report",
			"node": "roomba",
			"nodeId": "gatesroomba12",
			"productLine": "moscow",
			"roombareport": {
				"isDock": self.latest_dock_status,
				"action": action,
				"label":label,
				"isReady": isReady,
				"withDice": iswithDice,
				"isAtBase1": isAtBase1,
				"isAtBase2": isAtBase2,
				"isAtBase3": isAtBase3,
				"isMoving": isMoving,
				"position": [

							int(self.latest_pose_stamped.pose.position.x*1000),
							int(self.latest_pose_stamped.pose.position.y*1000),
							int(self.latest_pose_stamped.pose.position.z*1000)],

				"Fault": {
				}
			},
			"date": str(datetime.now())
		}
		
		mqttc.publish(topic, json.dumps(data))


	def dock_status_callback(self, msg):
		self.latest_dock_status = msg.data
		self.get_logger().info(f"Received /is_docked status: {self.latest_dock_status}")


	def pose_callback(self, msg):
		self.latest_pose_stamped = msg # NavToPosition needs the whole thang
		# self.get_logger().info(f"Received stamped pose status: {self.latest_pose_stamped}")


	def ir_opcode_callback(self, msg):
		print("msg", msg)
		print(type(msg))
		self.latest_ir_opcode = msg 
		print("msg.data:", self.latest_ir_opcode)


		# print("IrOpcode from within the callback:", self.latest_ir_opcode)
		print("DEBUG: All opcode information:", msg)
		self.get_logger().info(f"Received IrOpcode: {self.latest_ir_opcode}")


	def chirp(self, sent_notes):
		"""
		This function plays a given chirp tone.
		"""
		self.audio_ac.wait_for_server()

		goal = AudioNoteSequence.Goal()
		goal.iterations = 1
		goal.note_sequence = AudioNoteVector(notes=sent_notes)  # Wrap the notes in an AudioNoteVector

		# Send the goal
		self.audio_ac.send_goal_async(goal)


	##### Methods for movements #####
	def undock(self):
		# print("Sending report... \n")
		# self.reportSender("undock", isMoving=True)
		self.chirp(start_note)

		# Read current dock status, then undock
		self.dock_sensor.publish_dock_status() # Tells the publisher to update the dock status
		self.undock_ac.wait_for_server() # Wait till its ready
		undock_goal = Undock.Goal() # Make goal
		self.undock_ac.send_goal(undock_goal) # Send goal blocking
		
		self.dock_sensor.publish_dock_status() # Tells the publisher to update the dock status
		time.sleep(1)

		# print("Report sent \n")
		self.chirp(end_note)


	def dock(self):
		self.chirp(start_note)
		self.chirp(start_note)

		self.dock_ac.wait_for_server()
		dock_goal = Dock.Goal()
		self.dock_ac.send_goal(dock_goal)
		self.dock_sensor.publish_dock_status()
		time.sleep(1)
		self.chirp(end_note)


	def reset_pose(self):
		"""
		This function resets the robot's pose estimate.
		"""
		try:
			# Create a request object
			request = ResetPose.Request()

			# Call the service asynchronously
			response = self.reset_pose_srv.call_async(request)

			if response is not None:
				self.get_logger().info('ResetPose service called successfully.')
				time.sleep(1)
			else:
				self.get_logger().error('No response from ResetPose service.')

		except Exception as e:
			self.get_logger().error('Failed to call ResetPose service: %r' % (e,))


	def record_pose(self):
		"""
		Store the current pose for future use.
		"""
		odometry_sensor.publish_odometry()
		time.sleep(1) # Provides enough time to avoid errors, apparently
		if self.latest_pose_stamped is not None:
			self.recorded_pose = self.latest_pose_stamped
			self.get_logger().info("PoseStamped recorded:\n")
			
			# DEBUG: Check the coordinates
			# print(int(self.latest_pose_stamped.pose.position.x*1000))
			# print("\n", int(self.latest_pose_stamped.pose.position.y*1000))
			# print("\n", int(self.latest_pose_stamped.pose.position.z*1000))

		else:
			self.get_logger().error("No current pose available to record.")


	def drive_amnt(self, distance):
		print("Driving amount:", distance, "m")

		# print("Sending report... \n")
		# self.reportSender("undock", isMoving=True)

		self.chirp(start_note)
		self.drive_ac.wait_for_server()
		drive_goal = DriveDistance.Goal()
		drive_goal.distance = distance
		self.drive_ac.send_goal(drive_goal)
		
		time.sleep(1)  # Consider using async

		# print("Report sent \n")
		# self.reportSender("undock_done")
		self.chirp(end_note)


	def drive_amnt_async(self, distance):
		print("Driving amount:", distance, "m")

		self.chirp(start_note)
		self.drive_ac.wait_for_server()
		drive_goal = DriveDistance.Goal()
		drive_goal.distance = distance
		self.drive_ac.send_goal_async(drive_goal)
		
		time.sleep(2)
		self.chirp(end_note)


	def rotate_amnt(self, angle):
		print("Rotating amount:", angle, "rad")
		self.chirp(start_note)
		self.rotate_ac.wait_for_server()
		rotate_goal = RotateAngle.Goal()
		rotate_goal.angle = angle
		self.rotate_ac.send_goal(rotate_goal)
		
		time.sleep(1)  # Consider using async
		self.chirp(end_note)


	def rotate_amnt_async(self, angle):
		"""
		Rotate by a certain angle.
		"""
		print("Rotating amount:", angle, "rad")
		self.chirp(start_note)
		self.rotate_ac.wait_for_server()
		rotate_goal = RotateAngle.Goal()
		rotate_goal.angle = angle
		self.rotate_ac.send_goal_async(rotate_goal)
		
		time.sleep(2)
		self.chirp(end_note)


	def navigate_to_recorded_pose(self):
		"""
		Navigate back to the stored pose.
		"""
		self.chirp(start_note)
		
		print("Navigating back to home position from Odometry reading (PoseStamped)")
		if self.recorded_pose is not None:
			goal_msg = NavigateToPosition.Goal()
			goal_msg.goal_pose = self.recorded_pose
			goal_msg.achieve_goal_heading = True
			goal_msg.max_translation_speed = 0.3
			goal_msg.max_rotation_speed = 1.9

			self.nav_to_pos_ac.wait_for_server()
			self.nav_to_pos_ac.send_goal(goal_msg)
			self.chirp(ready_notes)
		else:
			self.chirp(error_notes)
			self.get_logger().error("No pose has been recorded.")


	def docking(self):
		while self.latest_dock_status != 'True':  # Ensuring the comparison is to a string if that's what's expected
			try:
				self.dock_sensor.publish_dock_status()
				self.dock_sensor.publish_dock_status()
				self.latest_ir_opcode = self.ir_sensor.publish_ir_opcode()

				# Add the latest opcode to the history
				print("Dock function IrOpcode readout:", self.latest_ir_opcode)
				print("Start- latest_dock_status:", self.latest_dock_status)

				# Check if the latest_ir_opcode has been updated
				if self.latest_ir_opcode is not None:
					if self.latest_dock_status == True:
						break

					if self.latest_ir_opcode == 160 or self.latest_ir_opcode == 161:
						# Reiterate, do nothing just continue the loop
						self.ir_opcode_history.append(self.latest_ir_opcode)
						self.get_logger().info(f"IR Opcode {self.latest_ir_opcode} seen, reiterating.")
						self.dock_sensor.publish_dock_status()


					elif self.latest_ir_opcode == 168:
						# Rotate right slightly
						self.rotate_amnt_async(pi/36)
						time.sleep(0.2)
						self.get_logger().info("Rotating right slightly due to Red Buoy detection.")
						self.ir_opcode_history.clear()  # Clear the history after movement
						self.dock_sensor.publish_dock_status()

					
					elif self.latest_ir_opcode == 164:
						# Rotate left slightly
						self.rotate_amnt_async(-pi/36)
						time.sleep(0.2)

						self.get_logger().info("Rotating left slightly due to Green Buoy detection.")
						self.ir_opcode_history.clear()  # Clear the history after movement
						self.dock_sensor.publish_dock_status()
					
					elif self.latest_ir_opcode == 172:
						# Drive forward a small amount
						self.drive_amnt_async(0.01)
						time.sleep(0.2)

						self.get_logger().info("Driving forward due to Red Buoy and Green Buoy detection.")
						self.ir_opcode_history.clear()  # Clear the history after movement
						self.dock_sensor.publish_dock_status()

				if len(self.ir_opcode_history) == self.ir_opcode_history.maxlen:
					self.drive_amnt(-0.3)
					self.navigate_to_recorded_pose()
					self.get_logger().info("Backing up due to continuous 160 or 161 opcodes.")
					self.ir_opcode_history.clear()  # Clear the history after movement

				# Print the current count of the deque
				print("Current opcode history count:", len(self.ir_opcode_history))
				self.dock_sensor.publish_dock_status()
				# print("END- latest_dock_status:", self.latest_dock_status)

				print("\n")
				time.sleep(0.3)  # Sleep for throttling
			except Exception as e:
				self.get_logger().error('Error in docking loop: {}'.format(e))
				break  # Or handle the exception appropriately


	def takeoff(self):
		try:
			
			# Define the process labels to be forwarded to the broker
			roomba_label_1 = "Traveling from base1 to base2."
			roomba_label_2 = "Traveling from base2 to base3."
			roomba_label_3 = "Traveling from base3 to base1."
			roomba_label_4 = "Finished cycle."

			### Actions for process 1: Navigating from base1 to base 2 ###
			# Undock and reset pose
			self.reportSender(roomba_label_1, action="undock_start", isAtBase1=True, isMoving=False)
			self.undock()
			self.reportSender(roomba_label_1, action="undock_done", isAtBase1=False, isMoving=True)

			# rotate amount
			self.reportSender(roomba_label_1, action="rotate_start", isMoving=True)
			self.rotate_amnt(-pi/2)
			self.reportSender(roomba_label_1, action="rotate_done", isMoving=True)

			# drive amount
			self.reportSender(roomba_label_1, action="drive_start", isMoving=True)
			self.drive_amnt(1.3)
			self.reportSender(roomba_label_1, action="drive_done", isMoving=True)

			# rotate amount
			self.reportSender(roomba_label_1, action="rotate_start", isMoving=True)
			self.rotate_amnt(pi/2)
			self.reportSender(roomba_label_1, action="rotate_done", isMoving=True)

			# drive amount
			self.reportSender(roomba_label_1, action="drive_start", isMoving=True)
			self.drive_amnt(2.0)
			self.reportSender(roomba_label_1, action="drive_done", isMoving=True)

			# dock
			self.reportSender(roomba_label_1, action="dock_start", isMoving=True)
			self.dock()
			self.reportSender(roomba_label_1, action="dock_done", isMoving=False, isAtBase2=True)
			time.sleep(3)  
			
			### !!!! Logic for waiting for beaker at base 2 goes here
			

			### Actions for process 2: Navigating to base3 from base2 ###
			self.reportSender(roomba_label_2, action="undock_start", isAtBase2=True, isMoving=False)
			self.undock()
			self.reportSender(roomba_label_2, action="undock_done", isAtBase2=False, isMoving=True)

			# drive amount
			self.reportSender(roomba_label_2, action="drive_start", isMoving=True)
			self.drive_amnt(2.65)
			self.reportSender(roomba_label_2, action="drive_done", isMoving=True)

			# rotate amount
			self.reportSender(roomba_label_2, action="rotate_start", isMoving=True)
			self.rotate_amnt(pi/2)
			self.reportSender(roomba_label_2, action="rotate_done", isMoving=True)

			# drive amount
			self.reportSender(roomba_label_2, action="drive_start", isMoving=True)
			self.drive_amnt(2.75)
			self.reportSender(roomba_label_2, action="drive_done", isMoving=True)

			# rotate amount
			self.reportSender(roomba_label_2, action="rotate_start", isMoving=True)
			self.rotate_amnt(pi/2)
			self.reportSender(roomba_label_2, action="rotate_done", isMoving=True)

			# drive amount
			self.reportSender(roomba_label_2, action="drive_start", isMoving=True)
			self.drive_amnt(2.5)
			self.reportSender(roomba_label_2, action="drive_done", isMoving=True)

			# dock
			self.reportSender(roomba_label_2, action="dock_start", isMoving=True)
			self.dock()
			self.reportSender(roomba_label_2, action="dock_done", isMoving=False, isAtBase3=True)

			### !!! Logic for communicating ready statuses goes here


			### Actions for process 3: Navigating to base1 from base3 ###
			self.reportSender(roomba_label_3, action="undock_start", isAtBase3=True, isMoving=False)
			self.undock()
			self.reportSender(roomba_label_3, action="undock_done", isAtBase3=False, isMoving=True)

			# drive amount
			self.reportSender(roomba_label_3, action="drive_start", isMoving=True)
			self.drive_amnt(2.65)
			self.reportSender(roomba_label_3, action="drive_done", isMoving=True)

			# rotate amount
			self.reportSender(roomba_label_3, action="rotate_start", isMoving=True)
			self.rotate_amnt(-pi/2)
			self.reportSender(roomba_label_3, action="rotate_done", isMoving=True)

			# drive amount
			self.reportSender(roomba_label_3, action="drive_start", isMoving=True)
			self.drive_amnt(2.60)
			self.reportSender(roomba_label_3, action="drive_done", isMoving=True)

			# rotate amount
			self.reportSender(roomba_label_3, action="rotate_start", isMoving=True)
			self.rotate_amnt(-pi/6)
			self.reportSender(roomba_label_3, action="rotate_done", isMoving=True)

			# drive amount
			self.reportSender(roomba_label_3, action="drive_start", isMoving=True)
			self.drive_amnt(1.75)
			self.reportSender(roomba_label_3, action="drive_done", isMoving=True)

			# rotate amount
			self.reportSender(roomba_label_3, action="rotate_start", isMoving=True)
			self.rotate_amnt(4*pi/6)
			self.reportSender(roomba_label_3, action="rotate_done", isMoving=True)

			# dock
			self.reportSender(roomba_label_3, action="dock_start", isMoving=True)
			self.dock()
			self.reportSender(roomba_label_3, action="dock_done", isMoving=False, isAtBase1=True)


		except Exception as error:
			roomba.chirp(error_notes)
			self.get_logger().error(f"Error in takeoff: {error}") # Error logging


if __name__ == '__main__':
	# rclpy.init()

	roomba = Roomba(namespace)
	exec = MultiThreadedExecutor(8)
	exec.add_node(roomba)
	exec.add_node(dock_sensor)
	exec.add_node(ir_sensor)
	exec.add_node(odometry_sensor)

	mqttc.loop_start()

	time.sleep(0.1)
	roomba.chirp(ready_notes)

	keycom = KeyCommander([
		(KeyCode(char='u'), roomba.takeoff),
		# (KeyCode(char='r'), roomba.rotate_amnt(pi/6)), # For debugging broker messages only
		# (KeyCode(char='s'), roomba.rotate_amnt(pi/6)), # For debugging broker messages only
	])
	print(" Press 'U' to intitiate launch")

	try:
		exec.spin()  # execute Roomba callbacks until shutdown or destroy is called
	except KeyboardInterrupt:
		print("KeyboardInterrupt, shutting down.")
	except Exception as error:
		print(f"Unexpected error: {error}")
	finally:
		exec.shutdown()
		roomba.destroy_node()
		rclpy.try_shutdown()

