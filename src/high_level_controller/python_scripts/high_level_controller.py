#!/usr/bin/env python
import rospy
import time
import random
from std_msgs.msg import Float32
from sensor_msgs.msg import Joy

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
		yaw_actuator_command_pub.publish(0)
		
	# check if pressing "Y"
	if data.buttons[3] == 1:
		# basically a go signal for predetermined action primitive
		yaw_actuator_command_pub.publish(1)


def main():
	# initialize node
	rospy.init_node('xbox2Arduino', anonymous = True)

	# initialize publishers
	global z_actuator_command_pub
	z_actuator_command_pub = rospy.Publisher('z_actuator_command', Float32, queue_size = 10)
	global yaw_actuator_command_pub
	yaw_actuator_command_pub = rospy.Publisher('yaw_actuator_command', Float32, queue_size = 10)

	# put both actuators to home positions
	z_actuator_command_pub.publish(0) 
	yaw_actuator_command_pub.publish(0) 
	time.sleep(3)

	# initialize joy subscriber
	global joySubscriber
	joySubscriber = rospy.Subscriber("joy", Joy, joy_callback)

	# rospy.spin()
	rospy.spin()

def parse_force_glove():
	# eventually parse force glove and send action primitives go signals. for now...
	pass

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass