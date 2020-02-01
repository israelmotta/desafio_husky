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


class Camera:
  mission_phase = None
  camera_info = None
  msg_move_to_goal = None
  flag = None
  timer_flag = None

  def __init__(self):
    # focal length
    self.focalLength = 937.8194580078125
    # Initialize the CvBridge class
    self.bridge = CvBridge()
    # timer var
    self.start = time.time()
    # Initialize the ROS Node named 'opencv_camera', allow multiple nodes to be run with this name
    rospy.init_node('opencv_camera', anonymous=True)
    # controllers
    self.linear_vel_control = Controller(5, -5, 0.01, 0, 0)
    self.angular_vel_control = Controller(5, -5, 0.01, 0, 0)
    # odometry topic subscription
    rospy.Subscriber('/odometry/filtered', Odometry, self.callback_odometry)
    # Initalize a publisher to the "/camera/param" topic with the function "image_callback" as a callback
    self.image_pub = rospy.Publisher('/camera/param', Image, queue_size=10)
    # get camera info
    rospy.Subscriber("/diff/camera_top/camera_info", CameraInfo, self.callback_camera_info)
    # move to goal 
    self.pub_move_to_goal = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
    self.msg_move_to_goal = PoseStamped()
    self.flag = True
    self.camera_info = CameraInfo()

    self.start_map = rospy.Publisher("/GetFirstMap/goal", GetFirstMapActionGoal, queue_size=1)
    self.start_explore = rospy.Publisher("/Explore/goal", ExploreActionGoal, queue_size = 1)
    self.cancel_map = rospy.Publisher("/GetFirstMap/cancel", GoalID, queue_size = 1)
    self.cancel_explore = rospy.Publisher("/Explore/cancel", GoalID, queue_size = 1)
    time.sleep(1)
    self.start_map.publish()
    time.sleep(5)
    self.cancel_map.publish()
    time.sleep(2)
    self.start_explore.publish()


  # Define a callback for the Image message
  def callback(self, img_msg):
    # setup timer and font
    timer = int(time.time() - self.start)
    font = cv2.FONT_HERSHEY_SIMPLEX


    # Try to convert the ROS Image message to a CV2 Image
    try:
        cv_image = self.bridge.imgmsg_to_cv2(img_msg, "passthrough")
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

    # Converts images from RGB to BGR 
    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    # Converts images from BGR to HSV 
    img_hsv = cv2.cvtColor(img_rgb, cv2.COLOR_BGR2HSV)
    
    # Here we are defining range of bluecolor in HSV 
    # This creates a mask of blue coloured  
    # objects found in the frame. 
    mask = cv2.inRange(img_hsv, rangomin, rangomax)

    # The bitwise and of the frame and mask is done so  
    # that only the blue coloured objects are highlighted  
    # and stored in res 
    kernel = np.ones((5 ,5), np.uint8)
    FrameBinarizado = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    # find contours of image (cv2.CHAIN_APPROX_SIMPLE is for memory saves)
    _, cnts, _ = cv2.findContours(FrameBinarizado.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # cv2.drawContours(img, cnts,-1,(255,0,255),3)

    contour_list = []    
    ### CIRCLE DETECTION ###    
    contours_poly = []
    centers = []
    radius = []     

    for index, c in enumerate(cnts):
        # If the area of the captured contour is small, nothing happens
        if cv2.contourArea(c) < AreaContourLimitMin:
            continue
        
        # area = cv2.contourArea(c)

        # Approximates a polygonal curve with the specified precision
        contours_poly.append(cv2.approxPolyDP(c,0.01*cv2.arcLength(c,True),True))

        (x, y, w, h) = cv2.boundingRect(c)   #x and y: coordinates of the upper left vertex
                                                #w and h: respectively width and height of the rectangle

        cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
      
        # Determines the center point of the contour and draws a circle to indicate
        CoordenadaXCentroContorno = int((x+x+w)/2)
        CoordenadaYCentroContorno = int((y+y+h)/2)
        PontoCentralContorno = (CoordenadaXCentroContorno,CoordenadaYCentroContorno)
        cv2.circle(img, PontoCentralContorno, 1, (0, 0, 0), 5)
        coordinates = [PontoCentralContorno[0],PontoCentralContorno[1]]


        centers.append(coordinates)
        radius.append(w)     


        # Check vertices
        if ((len(contours_poly[index]) > 8) & (len(contours_poly[index]) < 23)):

            
          ContourQty = ContourQty + 1


          cv2.putText(cv_image, 'BOMB HAS BEEN DETECTED!', (20, 130), font, 2, (0, 0, 255), 5)
          # Obtain contour coordinates (in fact, from a rectangle that can cover the entire contour) and
          #emphasizes the outline with a rectangle.
                   

          # Finds and draw a circle that indicates the contour
          # (a, b) = cv2.minEnclosingCircle(c) # a and b: center and radius of the circle respectively
          # rospy.loginfo("Center %d", b)
          # cv2.circle(img, int(a), int(b), (0, 255, 0), 5)
          
          # Pass coordinates x, y and radius of circle to a variable 
          
          self.goal_move_base(centers[0][0], radius[0], width)
    
    # merge timer info to frame
    cv2.putText(cv_image, str(timer) + 's', (20, 60), font, 2, (50, 255, 50), 5) 
    cv2.putText(cv_image, str(time.ctime()), (10, 700), font, 2, (50, 255, 50), 6)
    
    img_view = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    # convert img to ros and pub image in a topic
    msg_frame = self.bridge.cv2_to_imgmsg(img_view, "bgr8")
    self.image_pub.publish(msg_frame)

  def callback_odometry(self, data):
    self.odometry_data = data

  def callback_camera_info(self, data):
    self.camera_info = data

  def listener(self):
    # subscribe to a topic
    rospy.Subscriber('/diff/camera_top/image_raw', Image, self.callback)  
    # simply keeps python from exiting until this node is stopped
    rospy.spin()

  def cmd_vel_pub(self, linear, angular, frame):
    cv2.putText(frame, 'Process: center alignment', (20, 640), cv2.FONT_HERSHEY_SIMPLEX, 2, (200, 0, 0), 3)
    vel_msg = Twist()
    vel_msg.linear.x = linear
    vel_msg.angular.z = angular
    self.velocity_publisher.publish(vel_msg)
  
  def goal_move_base(self, center_ball, radius, image_size):
    distance = (1 * self.focalLength) / (radius * 2)
    y_move_base = -(center_ball - image_size/2) / (radius*2) 
    if abs(y_move_base) < 0.006:
      x_move_base = distance
    else:
      x_move_base = math.sqrt(distance**2 - y_move_base**2)
    self.msg_move_to_goal.pose.position.x = x_move_base - 2
    self.msg_move_to_goal.pose.position.y = y_move_base
    self.msg_move_to_goal.pose.orientation.w = 1
    self.msg_move_to_goal.header.frame_id = "camera"
    if self.flag:
      self.cancel_explore.publish()
      os.system("rosnode kill /explore")
      time.sleep(1)  
      self.pub_move_to_goal.publish(self.msg_move_to_goal)
      self.flag = False
      self.timer_flag = time.time()
    if time.time() - self.timer_flag > 5:
      self.flag = True      
    print('distance to sphere: ' + str(distance))
    print('INCREMENTO X: ' + str(x_move_base))
    print('INCREMENTO Y: ' + str(y_move_base))


  def pub_move_base(self, x, y):
    if self.mission_phase == None:
      self.mission_phase = 1

  #def move_base_pub(self, x, y, angle):
    #coment

class Controller:
  sat_max = 0
  sat_min = 0
  kp = 0
  ki = 0
  kd = 0
  error_integral = 0 
  error_prev = 0 

  def __init__ (self, sat_max, sat_min, kp, ki, kd):
    self.sat_max = sat_max 
    self.sat_min = sat_min 
    self.kp = kp 
    self.ki = ki 
    self.kd = kd 
    
  def calculate(self, time, setpoint, process):
    # set the error
    self.error = setpoint - process
    self.error_integral =+ self.error
    # calculate the output
    control_output = self.kp*self.error + self.ki*(self.error_integral)*time + self.kd*(self.error - self.error_prev)/time    
    # using saturation max and min in control_output 
    if (control_output > self.sat_max):
      control_output = self.sat_max
    elif (control_output < self.sat_min):
      control_output = self.sat_min
    # set error_prev for kd   
    self.error_prev = self.error   
    return control_output  

# main function
if __name__	== '__main__':
  try:
    cam_print = Camera()  
    cam_print.listener()  
  except rospy.ROSInterruptException:
    pass