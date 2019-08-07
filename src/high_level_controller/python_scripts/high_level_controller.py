#!/usr/bin/env python

import rospy
import time
import random
from std_msgs.msg import Float32
from sensor_msgs.msg import Joy

gripper_delay_move_flag = 0 # whether we should be delaying the grippers move
gripper_move_flag = 0 # whether we should be moving the gripper
gripper_go_home_flag = 0 # which direction to move gripper
gripper_go_open_door_flag = 0 # which direction to move gripper

def joy_callback(data):
	# check if pressing "A"
	if data.buttons[0] == 1:
		# basically a go signal for predetermined return to home position
		z_actuator_command_pub.publish(0)

	# check if pressing "B"
	if data.buttons[1] == 1:
		# basically a go signal for predetermined action primitive
		z_actuator_command_pub.publish(1)

	# check if pressing "X"
	if data.buttons[2] == 1:
		# basically a go signal for predetermined return to home position
		# yaw_actuator_command_pub.publish(0)
		gripper_move_flag = 1 # can start move with yaw axis
		gripper_go_home_flag = 1

	# check if pressing "Y"
	if data.buttons[3] == 1:
		# basically a go signal for predetermined action primitive
		# yaw_actuator_command_pub.publish(1)
		gripper_delay_move_flag = 1 # have to delay move
		gripper_go_open_door_flag = 1


def main():
	# initialize node
	rospy.init_node('xbox2Arduino', anonymous = True)

	# initialize publishers
	global z_actuator_command_pub
	z_actuator_command_pub = rospy.Publisher('z_actuator_command', Float32, queue_size = 10)
	global yaw_actuator_command_pub
	yaw_actuator_command_pub = rospy.Publisher('yaw_actuator_command', Float32, queue_size = 10)
	global gripper_command_pub
	gripper_command_pub = rospy.Publisher('gripper_command', Float32, queue_size = 10)

	# put both actuators to home positions
	#z_actuator_command_pub.publish(0)
	#yaw_actuator_command_pub.publish(0)
	time.sleep(3)

	# initialize joy subscriber
	global joySubscriber
	joySubscriber = rospy.Subscriber("joy", Joy, joy_callback)

	# rospy.spin()
	rospy.spin()

	# high level control loop
	control_loop()

def control_loop():
	# send send send
	readyCount = 0
	pubRate = 50 # hertz
	rate = rospy.Rate(pubRate) # hertz

	# gripper move variables
	gripper_delay_move_counter = 0
	gripper_move_counter = 0
	gripper_delay_move_time = 3 # how long to delay the gripper moving (seconds)
	gripper_move_time = 3 # how long to move the gripper for (seconds)

	while (not rospy.is_shutdown()):
		# check if the gripper is being delayed
		if gripper_delay_move_flag == 1:
			gripper_delay_move_counter = gripper_delay_move_counter + 1

		# check if the gripper should begin being moved
		if gripper_delay_move_counter >= gripper_delay_move_time*pubRate:
			gripper_move_flag = 1
			gripper_delay_move_counter = 0
			gripper_delay_move_flag = 0

		# move the gripper
		if gripper_move_flag == 1:
			if gripper_go_home_flag == 1:
				pass
				#gripper_command_pub.publish(0)
			if gripper_go_open_door_flag == 1:
				pass
				#gripper_command_pub.publish(1)
			gripper_move_counter = gripper_move_counter + 1

		# check if the gripper should still be being moved
		if gripper_move_counter >= gripper_move_time*pubRate :
			gripper_go_home_flag = 0
			gripper_go_open_door_flag = 0
			gripper_move_counter = 0
			gripper_move_flag = 0
			#gripper_command_pub.publish(2) # stop command

		# sleep to maintain loop rate
		rate.sleep()


def parse_force_glove():
	# eventually parse force glove and send action primitives go signals. for now...
	pass

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
