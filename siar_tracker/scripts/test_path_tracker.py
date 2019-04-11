#!/usr/bin/env python

PKG = 'siar_tracker'
import roslib; roslib.load_manifest(PKG)
import rospy
import tf
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist, PoseStamped
import numpy as np

if __name__ == '__main__':
  rospy.init_node("test_siar_path_tracker")
  frame_id = rospy.get_param('~frame_id', default="/base_link")    
  path_pub = rospy.Publisher('path',Path, queue_size=2)
  # Test path
  vec = np.loadtxt(filename)
  path = Path()
  path.header.frame_id = frame_id
  path.header.stamp = rospy.Time.now()
  
  curr_wp = PoseStamped()
  curr_wp.header = path.header
  curr_wp.pose.orientation.x = 0
  curr_wp.pose.orientation.y = 0
  curr_wp.pose.orientation.z = 0
  curr_wp.pose.orientation.w = 1
  curr_wp.pose.position.z = 0
  for row in vec:
      if len(row) >= 2:
          curr_wp.pose.position.x = row[0]
          curr_wp.pose.position.y = row[1]
  path_pub.publish(path)
  
  # Spin until ctrl + c  
  rospy.spin()