#!/usr/bin/python
import rospy
from sensor_msgs.msg import Range
import sys
import time
# using mraa for gpio: 1.it needs root access 2.Uses physical pin numbers for gpio
import mraa

# Global variables and setting up pins
# echo and trig pins. note that numbers are physical pins numbers
TRIG_NO = 16 # in GPIO is 23
ECHO_NO = 18 # in GPIO is 24
# before setting the in and out we need
# to export to GPIO for use(base on mraa tutorial
# on upboard wiki)
TRIG = mraa.Gpio(TRIG_NO)
ECHO = mraa.Gpio(ECHO_NO)
# base on mraa first example we need
# to wait for exports
time.sleep(0.1)
# set input and output mode of pins
TRIG.dir(mraa.DIR_OUT)
ECHO.dir(mraa.DIR_IN)
# initialization of message container to send via ros
distance_message = Range()

# Writing a function to determine the distance in each call
def determine_dist():
    # creating the trigger pulse
    TRIG.write(0)
    time.sleep(0.08) # turn off and wait for settle for clean pulse

    TRIG.write(1) # rising edge
    time.sleep(0.00001) # flat part of signal, 10 microseconds is from
                        # datasheet
    TRIG.write(0) # falling edge

    # now let's read the echo pin for determine the travel time of signal
    while (ECHO.read() == 0):
        pulse_start = time.time()
    while (ECHO.read() == 1):
        pulse_end = time.time()
    
    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150
    distance = round(distance, 2)
    # now return the determined distance
    return distance

# now let's define a function for publishing the data in ros
def distance_publisher():
    # publisher object
    pub = rospy.Publisher("/ultrasonic_distance", Range, queue_size=10)
    # node type is ok, so initialize it
    rospy.init_node("ultrasonic_distance_publisher", anonymous=True)
    rate = rospy.Rate(10) # rate regulizer object

    # loop to publish data continiously
    while not rospy.is_shutdown():
        distance_message.range = determine_dist()
        pub.publish(distance_message)
        rate.sleep()

    # this function has nothing to return
    # it just does a job
    return

# main function
if __name__ == '__main__':
    try:
        distance_publisher()
    except rospy.ROSInterruptException:
        pass

# that's it. this is the end!!!
