![alt text](https://raw.githubusercontent.com/israelmotta/desafio_husky/master/src/husky/fig/HuskyMap.png)


Challenge with the Husky carried out in the simulation of the Senai Cimatec yard, with the objective of finding and recognizing a yellow ball.


# Requirements

* Ubuntu 18.04 (Bionic): http://wiki.ros.org/melodic/Installation/Ubuntu
* ROS Melodic: http://wiki.ros.org/melodic
* Gazebo
---


# Setup

* Clone this repository:

  ```
  git clone https://github.com/israelmotta/desafio_husky.git
  ```
  
* Go to /src directory;

* Cimatec world:

  ```
  git clone https://github.com/PPVTecchio/cimatec_map.git
  ```

* Husky:

  ```
  git clone https://github.com/husky/husky.git
  ```

* Velodyne VLP-16:

  ```
  git clone https://bitbucket.org/DataspeedInc/velodyne_simulator.git
  ```

* Pointcloud to laserscan:

  ```
  git clone https://github.com/ros-perception/pointcloud_to_laserscan.git
  ```

* Nav2D:

  ```
  git clone https://github.com/skasperski/navigation_2d.git
  ```

* Execute:
 
  ```
  catkin_make
  source devel/setup.bash
  ```
  
 
 # Simulation
 
 * Put a yellow ball in a random part of the map;

 
 * Run:
 
  ```
  roslaunch desafio_husky world1.launch
  
  roslaunch desafio_husky mapping.launch
  
  cd desafio_husky/src/desafio/scripts
  python explore2.py
  ```
 
 * Wait until the robot find the ball.
 

# Packages used

* Husky: https://github.com/husky/husky
* Velodyne VLP-16: https://bitbucket.org/DataspeedInc/velodyne_simulator/src/master/
* Cimatec world: https://github.com/PPVTecchio/cimatec_map
* Pointcloud to laserscan: http://wiki.ros.org/pointcloud_to_laserscan
* Nav2D: http://wiki.ros.org/nav2d/Tutorials


# References

* Camera:

  * Camera:https://github.com/SMARTlab-Purdue/ros-tutorial-gazebo-simulation/wiki/Sec.-4:-Creating-a-light-sensor-plugin

* Detecção:

  * OpenCV with ROS: https://dabit-industries.github.io/turtlebot2-tutorials/14b-OpenCV2_Python.html
  * Detecting circles: https://www.pyimagesearch.com/2014/07/21/detecting-circles-images-using-opencv-hough-circles/
  * Circular shapes using contours: http://layer0.authentise.com/detecting-circular-shapes-using-contours.html
  * Find distance using OpenCV: https://www.pyimagesearch.com/2015/01/19/find-distance-camera-objectmarker-using-python-opencv/

* Husky:

  * Husky customization: https://www.clearpathrobotics.com/assets/guides/husky/HuskyGPSWaypointNav.html

