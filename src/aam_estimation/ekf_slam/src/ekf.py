#!/usr/bin/env python

import numpy as np
import math
import rospy
from aam_common_msgs.msg import Cone
from aam_common_msgs.msg import Map
from aam_common_msgs.msg import ConeDetections
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import sys
import ros_numpy
import time 
import math
import matplotlib.pyplot as plt 
from nav_msgs.msg import Odometry
import tf 
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import Imu
from ackermann_msgs.msg import AckermannDriveStamped




class start_global_map():

  def __init__(self, namespace='data_gathering'):

    rospy.init_node("ekf_slam", anonymous = True)

    rospy.Subscriber("/lidar_cone_detection_cones",ConeDetections,callback=self.cones_callback)
    rospy.Subscriber("/robot_control/command",AckermannDriveStamped,self.control_callback)
    rospy.Subscriber("/gps_velocity",Vector3Stamped,self.gps_callback)
    rospy.Subscriber("/sensor_imu_hector",Imu,self.imu_callback)
    

    self.start = True
    self.control_velocity = 0
    self.control_yaw = 0
    self.measurement_velocity = 0
    self.measurement_yaw = 0

    self.xEst = np.zeros((3, 1))
    self.PEst = np.eye(3)


  def cones_callback(self,cone_detections):

    self.cones_x = []
    self.cones_y = []
    self.cones_color = []

    for cone in cone_detections.cone_detections:
      
      self.cones_x.append(cone.position.x)
      self.cones_y.append(cone.position.y)
      self.cones_color.append(cone.color)


    i = 0

    Xcones_filterd = []
    Ycones_filterd = []

    while i <= len(self.cones_x)-1:
      xcomp = self.cones_x[i]
      ycomp = self.cones_y[i]
      #print(len(self.cones_x),i)
      j = 0
      while j<= len(self.cones_x)-1:

        xiter = self.cones_x[j]
        yiter = self.cones_y[j]

        if ( (xcomp-xiter)<0.2 or (xcomp-xiter)<-0.2 ) and ((ycomp-yiter)<0.2 or (ycomp-yiter)<-0.2):
          break

        else :
          Xcones_filterd.append(xcomp)
          Ycones_filterd.append(ycomp)

        j+=1
      i+=1
    

    #print(len(self.cones_x),len(self.cones_y))

    SLAM = ekf(Xcones_filterd,Ycones_filterd,self.cones_color,self.control_yaw,self.control_velocity,self.measurement_velocity,self.measurement_yaw,self.start,self.xEst,self.PEst)
    SLAM.main()
    self.start = False


  def control_callback(self,control_msg):
    self.control_yaw = control_msg.drive.steering_angle
    self.control_velocity = control_msg.drive.speed


  def gps_callback(self,gps_msg):
    self.measurement_velocity =  gps_msg.vector.x
    

  def imu_callback(self,imu_msg):

    Ox_imu = imu_msg.orientation.x
    Oy_imu = imu_msg.orientation.y
    Oz_imu = imu_msg.orientation.z
    Ow_imu = imu_msg.orientation.w

    orientation_list = [ Ox_imu , Oy_imu , Oz_imu , Ow_imu ]
    (roll, pitch, yaw)  = euler_from_quaternion(orientation_list)

    self.measurement_yaw = yaw





class ekf():


  def __init__(self, cones_x, cones_y,cones_color,control_yaw,control_velocity,measurement_velocity,measurement_yaw,start,xEst,PEst):


    self.lm_x = cones_x
    self.lm_y = cones_y
    self.lm_color = cones_color
    
    self.status = start

    self.xEst = xEst
    self.PEst = PEst

    self.control_yaw = control_yaw
    self.control_velocity = control_velocity
    self.u = np.array([[self.control_velocity, self.control_yaw]])

    self.measurement_velocity = measurement_velocity
    self.measurement_yaw = measurement_yaw


    # EKF state covariance
    self.Cx = np.diag([0.5, 0.5, np.deg2rad(30.0)])**2 # Change in covariance

    #  Simulation parameter
    self.Qsim = np.diag([0.2, np.deg2rad(1.0)])**2  # Sensor Noise
    self.Rsim = np.diag([1.0, np.deg2rad(10.0)])**2 # Process Noise

    self.MAX_RANGE = 20.0  # maximum observation range
    self.M_DIST_TH = 0.5  # Threshold of Mahalanobis distance for data association.
    self.STATE_SIZE = 3  # State size [x,y,yaw]
    self.LM_SIZE = 2  # LM state size [x,y]



  def main(self):

    ud = self.u

    # sort and filter landmarks
    z = self.get_landmarks()
    #print(self.status)



    '''
    Performs an iteration of EKF SLAM from the available information. 
    param xEst: the belief in last position
    param PEst: the uncertainty in last position
    param u:    the control function applied to the last position 
    param z:    measurements at this step
    '''
    self.ekf_slam(self.xEst,self.PEst,ud,z)
    






  def ekf_slam(self,xEst, PEst, u, z):


    S = self.STATE_SIZE
    '''
    # Predict
    xEst, PEst, G, Fx = self.predict(xEst, PEst, u)
    initP = np.eye(2)
    
    # Update
    xEst, PEst = self.update(xEst, PEst, u, z, initP)
    '''
    #return xEst, PEst

    return 0 , 0
    









  def motion_model(self,x, u):
    DT = 1
    F = np.array([[1.0, 0, 0],
                  [0, 1.0, 0],
                  [0, 0, 1.0]])
    f_ = np.squeeze(np.asarray(F))


    B = np.array([[DT * math.cos(x[2, 0]), 0],
                  [DT * math.sin(x[2, 0]), 0],
                  [0.0, DT]])
    B_ = np.squeeze(np.asarray(B))


    x_ = np.squeeze(np.asarray(x))
    u_ = np.squeeze(np.asarray(u))

    x = np.dot(f_,x_) + np.dot( B_,u_)
    return x




  def get_landmarks(self):
    
    z = np.zeros((0, 3))

    for i in range(len(self.lm_x)):
      x = self.lm_x[i]
      y = self.lm_y[i]
      d = math.sqrt(x**2 + y**2)
      angle = self.pi_2_pi(math.atan2(y, x))


      if d <= self.MAX_RANGE:
        zi = np.array([d, angle, i])
        z = np.vstack((z, zi))

    #print(z)
    return z




  def pi_2_pi(self,angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi






  def predict(self,xEst, PEst, u):
    pass


  
  def update(self,xEst, PEst, u, z, initP):
    pass


  





























if __name__ == '__main__':
  try:
    global_map = start_global_map()
    
  except rospy.ROSInterruptException:
    pass
  rospy.spin()
  
