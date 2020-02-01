#!/usr/bin/env python2.7
# Import ROS libraries and messages
import rospy,  time
from sensor_msgs.msg import Image

# Import OpenCV libraries and tools
import cv2
from nav2d_navigator.msg import ExploreActionGoal
import numpy as np


# Initialize the ROS Node named 'exploration', allow multiple nodes to be run with this name
rospy.init_node('exploration', anonymous=True)

# Initalize a publisher to the "/Explore/goal" topic with the function "image_callback" as a callback

rospy.loginfo("Hello ROS!")
rospy.Publisher('/GetFirstMap/goal', GoalID, queue_size=1).publish()
rospy.Publisher('/Explore/goal', ExploreActionGoal, queue_size=1).publish(ExploreActionGoal())
time.sleep(10)


# Loop to keep the program from shutting down unless ROS is shut down, or CTRL+C is pressed
while not rospy.is_shutdown():
    rospy.spin()