#!/usr/bin/env python 

# libraries:
import cv2
import math
import time
import rospy
import numpy as np
import os
from std_msgs.msg import Int32
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3
from cv_bridge import CvBridge, CvBridgeError
from move_base_msgs.msg import MoveBaseActionGoal
from geometry_msgs.msg import PoseStamped
from nav2d_navigator.msg import GetFirstMapActionGoal, ExploreActionGoal
from actionlib_msgs.msg import GoalID 

# from controlvision import ControlVision

from control_pid import ControlPid

class Camera:
  mission_phase = None
  camera_info = None
  msg_move_to_goal = None
  flag = True
  timer_flag = None
  control_pid_x = None
  control_pid_yaw = None
  pub_cmd_vel = None
  cont = 0
  mission = False
  control = False

  def __init__(self):
    # Focal length
    self.focalLength = 878 #937.8194580078125
    # Initialize the CvBridge class
    self.bridge = CvBridge()
    # Timer var
    self.start = time.time()
    # Initialize the ROS Node named 'opencv_camera', allow multiple nodes to be run with this name
    rospy.init_node('opencv_camera', anonymous=True)
    # Initalize a publisher to the "/camera/param" topic with the function "image_callback" as a callback
    self.image_pub = rospy.Publisher('/camera/param', Image, queue_size=1)
    self.pub_cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    # Get camera info
    rospy.Subscriber("/diff/camera_top/camera_info", CameraInfo, self.callback_camera_info)
    # Move to goal 
    self.pub_move_to_goal = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
    self.msg_move_to_goal = PoseStamped()
    # self.flag = True
    self.camera_info = CameraInfo()
    self.control_pid_x = ControlPid(5, -5, 0.005, 0, 0)
    self.control_pid_yaw = ControlPid(3, -3, 0.001, 0, 0)
    self.cancel_move_base = rospy.Publisher("/move_base/cancel", GoalID, queue_size=1)
    self.controller_flag = False
    # Nav2d setup
    self.start_map = rospy.Publisher("/GetFirstMap/goal", GetFirstMapActionGoal, queue_size=1)
    self.start_explore = rospy.Publisher("/Explore/goal", ExploreActionGoal, queue_size = 1)
    self.cancel_map = rospy.Publisher("/GetFirstMap/cancel", GoalID, queue_size = 1)
    self.cancel_explore = rospy.Publisher("/Explore/cancel", GoalID, queue_size = 1)
    self.cancel_move_base = rospy.Publisher("/move_base/cancel", GoalID, queue_size = 1)

    # Nav2D mapping and explore
    time.sleep(1)
    self.start_map.publish()
    time.sleep(5)
    self.cancel_map.publish()
    time.sleep(2)
    # Go to a specified position on map
    self.msg_move_to_goal.pose.position.x = 40
    self.msg_move_to_goal.pose.position.y = 0
    self.msg_move_to_goal.pose.orientation.w = 1
    self.msg_move_to_goal.header.frame_id = 'base_link' #self.camera_info.header.frame_id
    self.pub_move_to_goal.publish(self.msg_move_to_goal)
    self.start_explore.publish()
    self.timer_flag = time.time()

  # Define a callback for the Image message
  def callback(self, img_msg):
    # Setup timer and font
    timer = int(time.time() - self.start)
    font = cv2.FONT_HERSHEY_SIMPLEX


    # Try to convert the ROS Image message to a CV2 Image
    try:
        cv_image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
    except CvBridgeError, e:
        rospy.logerr("CvBridge Error: {0}".format(e))
    
    img = cv_image.copy()

    AreaContourLimitMin = 100  # This value is empirical. Adjust it to your needs

    # Obtaining the image dimensions
    height = np.size(img,0)
    width= np.size(img,1)
    ContourQty = 0

    # Image processing
    
    # Define range
    rangomin = np.array([25, 50, 50])
    rangomax = np.array([32, 255, 255]) # B, G, R

    # Converts images from BGR to HSV 
    img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    
    # Here we are defining range of bluecolor in HSV 
    # This creates a mask of blue coloured  
    # objects found in the frame. 
    mask = cv2.inRange(img_hsv, rangomin, rangomax)

    # The bitwise and of the frame and mask is done so  
    # that only the blue coloured objects are highlighted  
    # and stored in res 
    kernel = np.ones((5 ,5), np.uint8)
    FrameBinarizado = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    # Find contours of image (cv2.CHAIN_APPROX_SIMPLE is for memory saves)
    _, cnts, _ = cv2.findContours(FrameBinarizado.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    contour_list = []    
    ### CIRCLE DETECTION ###    
    contours_poly = []
    centers = []
    radius = []   
    coordinates = [-1, -1, -1]  

    for index, c in enumerate(cnts):
        # If the area of the captured contour is small, nothing happens
        if cv2.contourArea(c) < AreaContourLimitMin:
            continue
        
        # Approximates a polygonal curve with the specified precision
        contours_poly.append(cv2.approxPolyDP(c, 0.009 * cv2.arcLength(c, True), True))
        aux1, aux2 = cv2.minEnclosingCircle(contours_poly[index])
        centers.append(aux1)
        radius.append(aux2) 

        # Check vertices
        if ((len(contours_poly[index]) > 8) & (len(contours_poly[index]) < 23)):
          # Finds and draw a circle that indicates the contour
          cv2.circle(img, (int(centers[index][0]), int(centers[index][1])), int(radius[index]), (0, 0, 255), 5)   
         
          ContourQty = ContourQty + 1

          if self.flag:
            cv2.putText(img, 'SPHERE DETECTED', (20, 130), font, 2, (0, 0, 255), 5)
          else:
            if self.control:
              cv2.putText(img, 'CONTROL INITIALIZED', (20, 130), font, 2, (0, 0, 255), 5)
            else:
              if self.mission:
                cv2.putText(img, 'MISSION FINISHED', (20, 130), font, 2, (0, 0, 255), 5)
          
          # Pass coordinates x, y and radius of circle to a variable 
          self.goal_move_base(centers[0][0], radius[0], width)
    
    # Merge timer info to frame
    cv2.putText(img, str(timer) + 's', (20, 60), font, 2, (50, 255, 50), 5) 
    cv2.putText(img, str(time.ctime()), (10, 700), font, 2, (50, 255, 50), 6)
    
    img_view = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    # Convert img to ros and pub image in a topic
    msg_frame = self.bridge.cv2_to_imgmsg(img, "bgr8")
    self.image_pub.publish(msg_frame)

  def callback_camera_info(self, data):
    self.camera_info = data

  def listener(self):
    # Subscribe to a topic
    rospy.Subscriber('/diff/camera_top/image_raw', Image, self.callback)  
    # Simply keeps python from exiting until this node is stopped
    rospy.spin()

  def goal_move_base(self, center_ball, radius, image_size):
    self.flag = True
    distance = (1 * self.focalLength) / (radius*2)
    y_move_base = -(center_ball - image_size/2) / (radius*2) 
    
    if abs(y_move_base) < 0.006:
      x_move_base = distance
    else:
      x_move_base = math.sqrt(distance**2 - y_move_base**2)
    
    print(' ' + str(x_move_base) + ' ' + str(y_move_base))
  
    self.msg_move_to_goal.pose.position.x = x_move_base
    self.msg_move_to_goal.pose.position.y = y_move_base
    self.msg_move_to_goal.pose.orientation.w = 1
    self.msg_move_to_goal.header.frame_id = "camera"

    if self.cont == 0:
      self.pub_move_to_goal.publish(self.msg_move_to_goal)

    self.cont += 1
    print('' + str(self.cont))
    
    if self.cont == 400:
      self.cont = 0
    
    if distance < 3:
      self.controller_flag = True
      self.cancel_move_base.publish()
     
    if self.controller_flag:
      msg_twist = Twist()
      msg_twist.angular.z = self.control_pid_yaw.pid_calculate(1, image_size/2, center_ball)
      msg_twist.linear.x = self.control_pid_x.pid_calculate(1, 255, radius)
      self.pub_cmd_vel.publish(msg_twist)
      self.flag = False
      self.control = True
      if msg_twist.angular.z <= 0.001 and msg_twist.linear.x <= 0.001:
        self.control = False
        self.mission = True
        self.controller_flag = False
        
    print('distance to sphere: ' + str(distance))
    print('INCREMENTO X: ' + str(x_move_base))
    print('INCREMENTO Y: ' + str(y_move_base))

# Main function
if __name__	== '__main__':
  try:
    cam_print = Camera()  
    cam_print.listener()  
  except rospy.ROSInterruptException:
    pass