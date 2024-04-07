# ROS Imports
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

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


# Wes' packages and nodes
import my_interfaces
from my_interfaces.msg import ReadyStatus
from my_interfaces.srv import CheckReadiness
from ReadyStatusPublisherNode import ReadyStatusPublisherNode
from ReadinessTrackerNode import ReadinessTrackerNode


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

ready_status_publisher_node = ReadyStatusPublisherNode()
readiness_tracker_node = ReadinessTrackerNode()

class Roomba(Node):
	def __init__(self, namespace):
		super().__init__('roomba_node')

		# Initialize node objects within the class for node operations
		self.dock_sensor = dock_sensor
		self.ir_sensor = ir_sensor
		self.odometry_sensor = odometry_sensor
		self.ready_status_publisher_node = ready_status_publisher_node
		self.readiness_tracker_node = readiness_tracker_node

		# Subscriptions: 
		# Split up to compensate for noisy subscriptions
		cb_dockstatus = MutuallyExclusiveCallbackGroup() # Perhaps unneeded since the dock_status_node takes care of the dockstatus
		cb_ir = MutuallyExclusiveCallbackGroup()
		cb_pose = MutuallyExclusiveCallbackGroup()
		cb_ready_status = MutuallyExclusiveCallbackGroup()


		self.dock_status_sub_ = self.create_subscription(Bool, f'/{namespace}/check_dock_status', 
												self.dock_status_callback, 10, callback_group=cb_dockstatus)

		self.ir_opcode_sub_ = self.create_subscription(String, f'/{namespace}/ir_opcode_number', 
														self.ir_opcode_callback, qos_profile_sensor_data, callback_group=cb_ir)

		self.current_pose_sub_ = self.create_subscription(PoseStamped, f'/{namespace}/pose_stamped', 
												self.pose_callback, qos_profile_sensor_data, callback_group=cb_pose)

		self.ready_status_subscription_ = self.create_subscription(ReadyStatus, 'robot_ready_status', 
												self.ready_status_callback, 10, callback_group=cb_ready_status)
		

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

		# Service for checking the status of any robot
		self.client = self.create_client(CheckReadiness, 'check_readiness')

		while not self.client.wait_for_service(timeout_sec=1.0):
			self.get_logger().info('Service not available, waiting again...')
		print("Readiness Tracker is Available. ")

		# Variable Initialization
		self.latest_dock_status = True #########!!!!!!!!!!!!! Was None
		self.latest_pose_stamped = None
		self.latest_ir_opcode = None
		self.ir_opcode_history = deque(maxlen=20)  # A deque to store the history of opcodes in the auto-docking function

		# variable that stores the robot ready status information
		# Ready statuses are updated in the callback functions.
		self.latest_ready_status = None


	##############################################################################################
	""" General ready status services and publishing stuff starts here """			
	##############################################################################################		
	def ready_status_callback(self, msg):
		"""
		The other nodes don't have this function, yet. 
		I think they will need it in order to do their own wait_for_all_ready operations.
		"""
		# Update the latest ready statuses with the message received from the 'robot_ready_status' topic
		self.latest_ready_status = msg
		# self.roomba_status = self.latest_ready_status.roomba

		# DEBUGGING
		print("DEBUG: Start of RoombaNode callback...")
		print("msg.roomba_base2 : 	", msg.roomba_base2)
		print("msg.beaker : 		", msg.beaker)
		print("msg.beaker_conv :	",msg.beaker_conv)
		print("msg.bunsen_conv:  	",msg.bunsen_conv)
		print("msg.bunsen : 		", msg.bunsen)
		print("msg.roomba_base3 :	", msg.roomba_base3)
		print("End of RoombaNode callback \n")

		# self.get_logger().info(f"Received /robot_ready_status: {self.latest_ready_status}")


	def check_robot_status(self, other_robot, expected_status):
		self.get_logger().info(f"Checking if {other_robot} is {'ready' if expected_status else 'not ready'}.")

		while True:
			future = self.send_request(other_robot)
			rclpy.spin_until_future_complete(self, future)

			if future.result() is not None:
				actual_status = future.result().ready
				if actual_status == expected_status:
					print("SUCCESS")
					self.get_logger().info(f'{other_robot} status matches the expected status: {expected_status}.')
					break  # Exit the loop if the status matches
				else:
					# This message will now reflect the actual status received and the expected status.
					self.get_logger().info(f'{other_robot} status does not match. Expected: {expected_status}. Actual: {actual_status}. Retrying...')
					time.sleep(1)  # Wait for a bit before retrying
			else:
				self.get_logger().error(f'Exception while calling service: {future.exception()}')
				time.sleep(1)  # Wait for a bit before retrying in case of an exception

	def display_robot_statuses(self):
		# DUBUGGING: Used in key commander to display the robot states on keypress in the terminal
		try: 
			self.ready_status_publisher_node.display_robot_statuses()
		except:
			self.get_logger().error(f"Error in display: {error}") # Error logging

	def publish_robot_status(self):
		self.ready_status_publisher_node.publish_ready_status()
		self.get_logger().info('Published the updated ready status')


	##############################################################################################
	""" Robot-specific status checking starts here. """			
	##############################################################################################

	# IMPORTANT! Always set your robot's status before checking other robots
	# NOTE: Always set crx10 statuses to False after picking up dice block


	def check_base2(self):

		print("Setting roomba_base2 status to True")
		self.set_roomba_base2_true()

		print("Check if beaker is ready at base2")
		self.check_robot_status('beaker', True)

		print("!!! Both robots are ready !!!")

	def check_dice_block_handoff_base2(self):

		print("Setting roomba_base2 status to True")
		self.set_roomba_base2_true() # May be unneccessary since it should already be true

		print("Checking whether beaker has retrieved the block and moved away")
		self.check_robot_status('beaker', False)

		print("!!! Both robots are ready !!!")


	# def check_base3(self):

	# 	print("Setting roomba_base3 status to True")
	# 	self.set_roomba_base3_true()

	# 	print("Check if bunsen is ready at base3")
	# 	self.check_robot_status('bunsen', True)

	# 	print("!!! Both robots are ready !!!")

	# def check_dice_block_handoff_base3(self):

	# 	print("Setting roomba_base3 status to True")
	# 	self.set_roomba_base3_true() # May be unneccessary since it should already be true

	# 	# Check whether beaker has retrieved the dice block and moved away
	# 	print("Checking whether bunen has the block before moving to base1")
	# 	self.check_robot_status('bunsen', False)

	# 	print("!!! Both robots are ready !!!")



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
			self.reportSender(label=roomba_label_1, action="undock_start", isAtBase1=True, isMoving=False)
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

	exec.add_node(ready_status_publisher_node)
	exec.add_node(readiness_tracker_node)

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

# # The following Key Commander is used for manually checking outputs in the terminal
# if __name__ == '__main__':

# 	roomba = RoombaNode()
# 	exec = MultiThreadedExecutor(8)

# 	exec.add_node(roomba)
# 	exec.add_node(ready_status_publisher_node)
# 	exec.add_node(readiness_tracker_node)


# 	time.sleep(1.0)
	
# 	keycom = KeyCommander([
# 		(KeyCode(char='u'), roomba.main), 
# 		(KeyCode(char='v'), roomba.display_robot_statuses),
# 		(KeyCode(char='r'), roomba.set_roomba_base2_true),
# 		(KeyCode(char='d'), roomba.set_roomba_base2_false),
# 		(KeyCode(char='t'), roomba.set_roomba_base3_true),
# 		(KeyCode(char='f'), roomba.set_roomba_base3_false),

# 		(KeyCode(char='`'), roomba.check_base2),
# 		(KeyCode(char='1'), roomba.check_dice_block_handoff_base2),
# 		(KeyCode(char='2'), roomba.check_base3),
# 		(KeyCode(char='3'), roomba.check_dice_block_handoff_base3),


# 	])
# 	# print(" Press 'u' to intitiate main routine") # This will be the navigation code
# 	print(" Press 'u' to intitiate main routine") # TESTING W/ PASSING VARS (KEY COMM ISSUES)
# 	print(" Press 'v' to display all robot states in the text file")
# 	print(" Press 'r' to set the roomba_base2's status as 'True'")
# 	print(" Press 'd' to set the roomba_base2's status as 'False'")
# 	print(" Press 't' to set the roomba_base3's status as 'True'")
# 	print(" Press 'f' to set the roomba_base3's status as 'False'")
# 	print(" Press 'c' to check the readiness of beaker")


# 	try:
# 		exec.spin()  # execute Roomba callbacks until shutdown or destroy is called
# 	except KeyboardInterrupt:
# 		print("KeyboardInterrupt, shutting down.")
# 	except Exception as error:
# 		print(f"Unexpected error: {error}")
# 	finally:
# 		exec.shutdown()
# 		roomba.destroy_node()
# 		rclpy.try_shutdown()

