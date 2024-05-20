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




class start_global_map():

  def __init__(self, namespace='data_gathering'):

    rospy.init_node("ekf_slam", anonymous = True)

    rospy.Subscriber("/lidar_cone_detection_cones",ConeDetections,callback=self.cones_callback)
    rospy.Subscriber("/robot_control/odom",Odometry,self.odometry_callback)
    rospy.Subscriber("/gps_velocity",Vector3Stamped,self.gps_callback)
    rospy.Subscriber("/sensor_imu_hector",Imu,self.imu_callback)
    


    self.control_velocity = 0
    self.control_yaw = 0
    self.measurement_velocity = 0
    self.measurement_yaw = 0


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

    SLAM = ekf(Xcones_filterd,Ycones_filterd,self.cones_color,self.control_yaw,self.control_velocity,self.measurement_velocity,self.measurement_yaw)
    SLAM.main()


  def odometry_callback(self,odom_msg):
    self.control_velocity = odom_msg.twist.twist.linear.x

    orientation_q = odom_msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw)  = euler_from_quaternion(orientation_list)

    self.control_yaw = yaw

    #print(self.carX , self.carY , self.car_yaw )


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


  def __init__(self, cones_x, cones_y,cones_color,control_yaw,control_velocity,measurement_velocity,measurement_yaw):


    self.lm_x = cones_x
    self.lm_y = cones_y
    self.lm_color = cones_color

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
    

    # State Vector [x y yaw v]'
    xEst = np.zeros((self.STATE_SIZE, 1))
    xTrue = np.zeros((self.STATE_SIZE, 1))
    PEst = np.eye(self.STATE_SIZE)

    xDR = np.zeros((self.STATE_SIZE, 1))  # Dead reckoning

    # history
    hxEst = xEst
    hxTrue = xTrue
    hxDR = xTrue

    # true car pose based on velocity model
    xTrue = self.motion_model(xTrue, self.u)

    # sort and filter landmarks
    landmarks = self.get_landmarks()

    # update and prediction step
    self.ekf_slam(xEst,landmarks,PEst)

    

    

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




  def ekf_slam(self,xEst,z,PEst):
    """
    Performs an iteration of EKF SLAM from the available information. 
    
    :param xEst: the belief in last position
    :param PEst: the uncertainty in last position
    :param u:    the control function applied to the last position 
    :param z:    measurements at this step
    :returns:    the next estimated position and associated covariance
    """

    S = self.STATE_SIZE

    # Predict
    xEst, PEst, G, Fx = self.predict(xEst, PEst, self.u)

    initP = np.eye(2)

    # Update
    xEst, PEst = self.update(xEst, PEst, self.u, z, initP)
    print(xEst,PEst)

    return
    




  def predict(self,xEst, PEst, u):
    """
    Performs the prediction step of EKF SLAM
    
    :param xEst: nx1 state vector
    :param PEst: nxn covariacne matrix
    :param u:    2x1 control vector
    :returns:    predicted state vector, predicted covariance, jacobian of control vector, transition fx
    """

    S = self.STATE_SIZE
    G, Fx = self.jacob_motion(xEst[0:S], u)
    
    xEst = self.motion_model(xEst, u)
    #print(xEst)


    # Fx is an an identity matrix of size (STATE_SIZE)
    # sigma = G*sigma*G.T + Noise
    PEst = np.dot(np.dot(G.T , PEst) , G )+ np.dot(np.dot(Fx.T , self.Cx ) , Fx)
    return xEst, PEst, G, Fx
    



  def jacob_motion(self,x, u):
    """
    Calculates the jacobian of motion model. 
    
    :param x: The state, including the estimated position of the system
    :param u: The control function
    :returns: G:  Jacobian
              Fx: STATE_SIZE x (STATE_SIZE + 2 * num_landmarks) matrix where the left side is an identity matrix
    """
    
    DT = 1

    # [eye(3) [0 x y; 0 x y; 0 x y]]
    Fx = np.hstack((np.eye(self.STATE_SIZE), np.zeros(
        (self.STATE_SIZE, self.LM_SIZE * self.calc_n_LM(x)))))

    jF = np.array([[0.0, 0.0, -DT * u[0] * math.sin(x[2, 0])],
                   [0.0, 0.0, DT * u[0] * math.cos(x[2, 0])],
                   [0.0, 0.0, 0.0]])

    G = np.eye(self.STATE_SIZE) + np.dot(np.dot(Fx.T,jF) , Fx)
    if self.calc_n_LM(x) > 0:
        print(Fx.shape)
    return G, Fx




  def calc_n_LM(self,x):
    """
    Calculates the number of landmarks currently tracked in the state
    :param x: the state
    :returns: the number of landmarks n
    """
    n = int((len(x) - self.STATE_SIZE) / self.LM_SIZE)
    return n





  def update(self,xEst, PEst, u, z, initP):
    """
    Performs the update step of EKF SLAM
    
    :param xEst:  nx1 the predicted pose of the system and the pose of the landmarks
    :param PEst:  nxn the predicted covariance
    :param u:     2x1 the control function 
    :param z:     the measurements read at new position
    :param initP: 2x2 an identity matrix acting as the initial covariance
    :returns:     the updated state and covariance for the system
    """
    for iz in range(len(z[:, 0])):  # for each observation
      minid = self.search_correspond_LM_ID(xEst, PEst, z[iz, 0:2]) # associate to a known landmark

      nLM = self.calc_n_LM(xEst) # number of landmarks we currently know about
      
      if minid == nLM: # Landmark is a NEW landmark
          print("New LM")
          # Extend state and covariance matrix
          xAug = np.vstack((xEst, self.calc_LM_Pos(xEst, z[iz, :])))
          PAug = np.vstack((np.hstack((PEst, np.zeros((len(xEst), self.LM_SIZE)))),
                            np.hstack((np.zeros((self.LM_SIZE, len(xEst))), initP))))
          xEst = xAug
          PEst = PAug
      
      lm = self.get_LM_Pos_from_state(xEst, minid)
      y, S, H = self.calc_innovation(lm, xEst, PEst, z[iz, 0:2], minid)

      K = np.dot(np.dot(PEst , H.T) , np.linalg.inv(S)) # Calculate Kalman Gain
      xEst = xEst + np.dot(K , y)
      PEst = (np.eye(len(xEst)) - np.dot(np.dot(K , H)) , PEst)
    
    xEst[2] = self.pi_2_pi(xEst[2])
    return xEst, PEst




  def search_correspond_LM_ID(self,xAug, PAug, zi):
    """
    Landmark association with Mahalanobis distance.
    
    If this landmark is at least M_DIST_TH units away from all known landmarks, 
    it is a NEW landmark.
    
    :param xAug: The estimated state
    :param PAug: The estimated covariance
    :param zi:   the read measurements of specific landmark
    :returns:    landmark id
    """

    nLM = self.calc_n_LM(xAug)

    mdist = []

    for i in range(nLM):
        lm = self.get_LM_Pos_from_state(xAug, i)
        y, S, H = self.calc_innovation(lm, xAug, PAug, zi, i)
        mdist.append(np.dot(np.dot(y.T , np.linalg.inv(S)) , y))

    mdist.append(self.M_DIST_TH)  # new landmark

    minid = mdist.index(min(mdist))

    return minid




  def calc_n_LM(self,x):
    """
    Calculates the number of landmarks currently tracked in the state
    :param x: the state
    :returns: the number of landmarks n
    """
    n = int((len(x) - self.STATE_SIZE) / self.LM_SIZE)
    return n




  def calc_LM_Pos(self,x, z):
    """
    Calcualtes the pose in the world coordinate frame of a landmark at the given measurement. 

    :param x: [x; y; theta]
    :param z: [range; bearing]
    :returns: [x; y] for given measurement
    """
    zp = np.zeros((2, 1))

    zp[0, 0] = x[0, 0] + z[0] * math.cos(x[2, 0] + z[1])
    zp[1, 0] = x[1, 0] + z[0] * math.sin(x[2, 0] + z[1])
    #zp[0, 0] = x[0, 0] + z[0, 0] * math.cos(x[2, 0] + z[0, 1])
    #zp[1, 0] = x[1, 0] + z[0, 0] * math.sin(x[2, 0] + z[0, 1])

    

    return zp




  def get_LM_Pos_from_state(self,x, ind):
    """
    Returns the position of a given landmark
    
    :param x:   The state containing all landmark positions
    :param ind: landmark id
    :returns:   The position of the landmark
    """
    lm = x[self.STATE_SIZE + self.LM_SIZE * ind: self.STATE_SIZE + self.LM_SIZE * (ind + 1), :]

    return lm






  def calc_innovation(self,lm, xEst, PEst, z, LMid):
      """
      Calculates the innovation based on expected position and landmark position
      
      :param lm:   landmark position
      :param xEst: estimated position/state
      :param PEst: estimated covariance
      :param z:    read measurements
      :param LMid: landmark id
      :returns:    returns the innovation y, and the jacobian H, and S, used to calculate the Kalman Gain
      """
      delta = lm - xEst[0:2]
      q = np.dot(delta.T , delta)[0, 0]
      zangle = math.atan2(delta[1, 0], delta[0, 0]) - xEst[2, 0]
      zp = np.array([[math.sqrt(q), self.pi_2_pi(zangle)]])
      # zp is the expected measurement based on xEst and the expected landmark position
      
      y = (z - zp).T # y = innovation
      y[1] = self.pi_2_pi(y[1])
      
      H = self.jacobH(q, delta, xEst, LMid + 1)
      S = np.dot(np.dot(H , PEst) , H.T) + self.Cx[0:2, 0:2]

      return y, S, H




  def jacobH(self,q, delta, x, i):
    """
    Calculates the jacobian of the measurement function
    
    :param q:     the range from the system pose to the landmark
    :param delta: the difference between a landmark position and the estimated system position
    :param x:     the state, including the estimated system position
    :param i:     landmark id + 1
    :returns:     the jacobian H
    """
    sq = math.sqrt(q)
    G = np.array([[-sq * delta[0, 0], - sq * delta[1, 0], 0, sq * delta[0, 0], sq * delta[1, 0]],
                  [delta[1, 0], - delta[0, 0], - q, - delta[1, 0], delta[0, 0]]])

    G = G / q
    nLM = self.calc_n_LM(x)
    F1 = np.hstack((np.eye(3), np.zeros((3, 2 * nLM))))
    F2 = np.hstack((np.zeros((2, 3)), np.zeros((2, 2 * (i - 1))),
                    np.eye(2), np.zeros((2, 2 * nLM - 2 * i))))

    F = np.vstack((F1, F2))

    H = np.dot(G , F)

    return H





if __name__ == '__main__':
  try:
    global_map = start_global_map()
    
  except rospy.ROSInterruptException:
    pass
  rospy.spin()
  
