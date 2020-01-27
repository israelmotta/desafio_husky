#!/usr/bin/env python2.7
# Import ROS libraries and messages
import rospy
from sensor_msgs.msg import Image

# Import OpenCV libraries and tools
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np


# Initialize the ROS Node named 'opencv_example', allow multiple nodes to be run with this name
rospy.init_node('opencv_example', anonymous=True)

# Initialize the CvBridge class
bridge = CvBridge()

# Define a function to show the image in an OpenCV Window
def show_image(img):
    cv2.imshow("Image Window", img)
    cv2.waitKey(3)

# Define a callback for the Image message
def image_callback(img_msg):
    # log some info about the image topic
    rospy.loginfo(img_msg.header)

    # Try to convert the ROS Image message to a CV2 Image
    try:
        cv_image = bridge.imgmsg_to_cv2(img_msg, "passthrough")
    except CvBridgeError, e:
        rospy.logerr("CvBridge Error: {0}".format(e))

    # show_image(cv_image)

    # Copy the image
    img = cv_image.copy()

    # Convert to grayscale
    gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    AreaContourLimitMin = 5000  # This value is empirical. Adjust it to your needs

    # Obtencao das dimensoes da imagem
    height = np.size(img,0)
    width= np.size(img,1)
    ContourQty = 0
     
    #tratamento da imagem
    
    # Define range
    rangomin = np.array([30, 50, 50])
    rangomax = np.array([50, 255, 255]) # B, G, R

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

    # show_image(FrameBinarizado)

    _, cnts, _ = cv2.findContours(FrameBinarizado.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # cv2.drawContours(img, cnts,-1,(255,0,255),3)

    contour_list = []

    for c in cnts:
        #se a area do contorno capturado for pequena, nada acontece
        if cv2.contourArea(c) < AreaContourLimitMin:
            continue
        
        # area = cv2.contourArea(c)

        approx = cv2.approxPolyDP(c,0.01*cv2.arcLength(c,True),True)

        if ((len(approx) > 8) & (len(approx) < 23)):


            ContourQty = ContourQty + 1

            #obtem coordenadas do contorno (na verdade, de um retangulo que consegue abrangir todo ocontorno) e
            #realca o contorno com um retangulo.
            (x, y, w, h) = cv2.boundingRect(c)   #x e y: coordenadas do vertice superior esquerdo
                                                #w e h: respectivamente largura e altura do retangulo

            cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
      
            #determina o ponto central do contorno e desenha um circulo para indicar
            CoordenadaXCentroContorno = int((x+x+w)/2)
            CoordenadaYCentroContorno = int((y+y+h)/2)
            PontoCentralContorno = (CoordenadaXCentroContorno,CoordenadaYCentroContorno)
            cv2.circle(img, PontoCentralContorno, 1, (0, 0, 0), 5)

            (a, b) = cv2.minEnclosingCircle(c)
            rospy.loginfo("Center %d", b)
            cv2.circle(img, PontoCentralContorno, int(b), (0, 0, 0), 5)



    if (ContourQty > 0):
        cv2.line(img, PontoCentralContorno,(int(width/2),CoordenadaYCentroContorno),(0,255,0),1)

    img_view = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    # Resize image to show    
    img_res = cv2.resize(img_view, (width/2, height/2)) 
    # Show the converted image
    show_image(img_res)


# Initalize a subscriber to the "/camera/rgb/image_raw" topic with the function "image_callback" as a callback
sub_image = rospy.Subscriber("/diff/camera_top/image_raw", Image, image_callback)

# Initialize an OpenCV Window named "Image Window"
cv2.namedWindow("Image Window", 1)

# Loop to keep the program from shutting down unless ROS is shut down, or CTRL+C is pressed
while not rospy.is_shutdown():
    rospy.spin()