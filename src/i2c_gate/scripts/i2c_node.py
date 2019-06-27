#!/usr/bin/python


# now let's go. i wanna change the code for using as a node. so import the rospy and the type of the 
# message.
import smbus
import time 
import rospy
from auto_msgs.msg import auto_com # this is the message type that we wrote it base on the geometry_msgs/Vector3 message.

# Initialize the I2C communication Bus

bus = smbus.SMBus(1)

# Address of the Arduino Board

mwb_add = 0x8

# The data containers

#auto_data = [1100, 1100, 1100]
auto_data_bytes = [0x4, 0x4c, 0x4, 0x4c, 0x4, 0x4c] # this is binarized auto_data

# the call back function that we will use for sending the subscribed data to midware board.
def callback(data):
	#auto_data = data.auto_com # this is the content of the message [roll pitch throttle]
	auto_data = [data.auto_com.x, data.auto_com.y, data.auto_com.z]
	# now converting the auto_data to 6 byte for sending via i2c
	for i in range(6):
		if (i%2 == 0):
			# note that because the Vector3 data type is float we must 
			# convert the data to integer to able to use bit shifting
			# operands.
			auto_data_bytes[i] = ((int(auto_data[(i//2)]) >> 8) & 0xFF)
		else:
			auto_data_bytes[i] = int(auto_data[(i//2)]) & 0xFF

	# now we will send the bytes via i2c
	bus.write_i2c_block_data(mwb_add, auto_data_bytes[0], auto_data_bytes[1:])

# now we define the listener function that creates the ros node for subscriber
def i2c_send():
	rospy.init_node('i2c_sender', anonymous=True)
	rospy.Subscriber("auto_command", auto_com, callback)
	rospy.spin()

# now the main function of the code 

if __name__ == '__main__':
	i2c_send()

# so i think thats it . so let's go ...


"""
# while loop for sending data continusly

#state = True

while(state):
	
	try:	

		# prepairing data to send
		auto_data_bytes[0] = ((auto_data[0] >> 8) & 0xFF)
		auto_data_bytes[1] = auto_data[0] & 0xFF
		auto_data_bytes[2] = ((auto_data[1] >> 8) & 0xFF)
		auto_data_bytes[3] = auto_data[1] & 0xFF
		auto_data_bytes[4] = ((auto_data[2] >> 8) & 0xFF)
		auto_data_bytes[5] = auto_data[2] & 0xFF
	
	
		# now we will send the bytes
		
		bus.write_i2c_block_data(mwb_add, auto_data_bytes[0], auto_data_bytes[1:])

		# changing the throttle for test

		if (auto_data[2] <= 1900):
			auto_data[2] += 100
		else:
			auto_data[2] = 1100

		# Delay for visualization in test
		time.sleep(2)

	except keyboardInterrupt:
		state = False
"""
# i think this would be ok . so let's go ...


