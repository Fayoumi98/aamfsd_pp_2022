#!/usr/bin/env python



import numpy as np
import math
import rospy
from sensor_msgs import msg
import tf
from aam_common_msgs.msg import Cone
from aam_common_msgs.msg import ConeDetections
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from sensor_msgs.msg import LaserScan
import sys
import matplotlib.pyplot as plt








class lidar_cone_detection:
  def __init__(self, namespace='lidar'):
    
    rospy.init_node("lidar_cone_detection", anonymous = True)


    rospy.Subscriber("/rplidar_points", LaserScan, self.lidarCallback)

    self.lidar_cone_detection_pub = rospy.Publisher("/lidar_cone_detection_cones",ConeDetections,queue_size=0)
    self.rviz_pub_array = rospy.Publisher("/lidar_cones_rviz",MarkerArray,queue_size=0)



    self.x_cones = []
    self.y_cones = []

    self.headingVector = []

    self.front_x_cones = []
    self.front_y_cones = []

    self.rviz_msg = []
    self.rviz_msg_array = []






  def lidarCallback(self,pc_msg):

    distance_array = []
    theta_array = []

    min_range = pc_msg.range_min
    max_range = pc_msg.range_max

    min_angle = pc_msg.angle_min
    max_angle = pc_msg.angle_max
    angle_increment = pc_msg.angle_increment

    #print(min_angle,"           ",max_angle,"          ",angle_increment)

    ranges = pc_msg.ranges

    j = 0

    while min_angle < max_angle:
        for i in ranges:
          if i < 719 and not math.isinf(i):
            distance_array.append(i)
            theta_array.append(min_angle)
            min_angle+= angle_increment
            #print("Distance :   ", distance[j], "   Theta :   " ,theta[j])
            j+=1
          else:
            min_angle != max_angle
            min_angle+= angle_increment
    
    x_yellow_list = []
    y_yellow_list = []
    x_blue_list = []
    y_blue_list = []

    lidar_cone_detection_msg = ConeDetections()

    f = 0

    while f < len(distance_array):

        theta = float(theta_array[f])
        distance = distance_array[f]
        #print(distance, theta)
        
        if theta < 0 :
          x_yellow = distance*math.cos(theta)
          y_yellow = distance*math.sin(theta)
          x_yellow_list.append(x_yellow)
          y_yellow_list.append(y_yellow)

        elif theta > 0 :
          x_blue = distance*math.cos(theta)
          y_blue = distance*math.sin(theta)
          x_blue_list.append(x_blue)
          y_blue_list.append(y_blue)
          
        f+=1
    

    
    x_yellow_cones = []
    y_yellow_cones = []

    x_blue_cones = []
    y_blue_cones = []



    for one , two in zip(x_yellow_list,y_yellow_list):
        count_yellow = 0
        cond_y = True
        while count_yellow < len(x_yellow_list)-1:

          if (one - x_yellow_list[count_yellow])==0:
            break

          elif abs(abs(one) - abs(x_yellow_list[count_yellow]))<0.05   or abs(abs(two) - abs(y_yellow_list[count_yellow]))<0.0009:
            cond_y = False
            #print(" first  condition yellow",abs(one - x_blue_list[count_yellow])," second condition yellow",abs(two - y_yellow_list[count_yellow]))
          
          count_yellow +=1

        if cond_y ==True:
          cone_msg = Cone()

          x_yellow_cones.append(one)
          y_yellow_cones.append(two)

          cone_msg.position.x = one
          cone_msg.position.y = two
          cone_msg.color.data = "yellow_cone"

          lidar_cone_detection_msg.cone_detections.append(cone_msg)
      
    #print(x_yellow_cones,"         ",y_yellow_cones)



      

    for one , two in zip(x_blue_list,y_blue_list):
      count_blue = 0
      cond_b = True
      while count_blue < len(x_blue_list)-1:
        #print(abs(two - y_blue_list[count_blue]))
        if (one - x_blue_list[count_blue])==0:
          break

        elif abs(abs(one) - (x_blue_list[count_blue]))<0.05   or abs(abs(two) - abs(y_blue_list[count_blue]))<0.0005:
          cond_b = False
            #print(" first  condition",abs(one - x_blue_list[count_blue])," second condition",abs(two - y_blue_list[count_blue]))
        
        count_blue +=1

      if cond_b ==True:
        cone_msg = Cone()

        x_blue_cones.append(one)
        y_blue_cones.append(two)

        cone_msg.position.x = one
        cone_msg.position.y = two
        cone_msg.color.data = "blue_cone"


        lidar_cone_detection_msg.cone_detections.append(cone_msg)

    #print(x_blue_cones,"         ",y_blue_cones)

    plt.scatter(x_yellow_cones,y_yellow_cones,c='yellow')
    plt.scatter(x_blue_cones,y_blue_cones,c='blue')
    #plt.show()

    self.lidar_cone_detection_pub.publish(lidar_cone_detection_msg)

    
    x_cones = []
    y_cones = []

 
    
    for x ,y in zip(x_yellow_cones,y_yellow_cones):
        x_cones.append(x)
        y_cones.append(y)
      

    for x ,y in zip(x_blue_cones,y_blue_cones):
        x_cones.append(x)
        y_cones.append(y)
    
    c = 0

    self.rviz_msg = Marker()
    self.rviz_msg_array = MarkerArray()

    while c < len(x_cones):
      
      
      x_cone = x_cones[c]
      y_cone = y_cones[c]


      c +=1
      count = 0
      MARKERS_MAX = 100

      self.rviz_msg = Marker()
      self.rviz_msg.header.frame_id = "map"
      self.rviz_msg.ADD
      self.rviz_msg.SPHERE
      self.rviz_msg.pose.position.x = x_cone
      self.rviz_msg.pose.position.y = y_cone
      self.rviz_msg.pose.position.z = 0

      self.rviz_msg.pose.orientation.w = 1
      self.rviz_msg.scale.x = 1
      self.rviz_msg.scale.y = 1
      self.rviz_msg.scale.z = 1
      self.rviz_msg.color.a = 1
      self.rviz_msg.color.r = 0
      self.rviz_msg.color.g = 0
      self.rviz_msg.color.b = 0
      self.rviz_msg.mesh_resource = "package://aamfsd_description/meshes/cone_blue.dae"
      self.rviz_msg.type = Marker.MESH_RESOURCE
      self.rviz_msg.mesh_use_embedded_materials = True


      if(count > MARKERS_MAX):
        self.rviz_msg_array.markers.pop(0)
      
      self.rviz_msg_array.markers.append(self.rviz_msg)
      m = 0
      id = 0
      for m in self.rviz_msg_array.markers:
        m.id = id
        id += 1
      f += 1
    
    self.rviz_pub_array.publish(self.rviz_msg_array)








if __name__ == '__main__':
  try:
      lidar = lidar_cone_detection()
  except rospy.ROSInterruptException:
      pass
  rospy.spin()
  
