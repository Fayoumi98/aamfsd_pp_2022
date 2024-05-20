#!/usr/bin/env python  

import rospy
import tf

if __name__ == '__main__':
  rospy.init_node('map_broadcaster')
  br_map_lidar = tf.TransformBroadcaster()
  br_odom = tf.TransformBroadcaster()

  while not rospy.is_shutdown():
      br_map_lidar.sendTransform((0.0, 0.0, 0.0),
                      (0.0, 0.0, 0.0, 1.0),
                      rospy.Time.now(),
                      "map",
                      "rplidar")

