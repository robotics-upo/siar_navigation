#!/usr/bin/env python

PKG = 'siar_tracker'
import roslib; roslib.load_manifest(PKG)
import rospy
import tf
import sys
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist, PoseStamped
import numpy as np

if __name__ == '__main__':
  if (len(sys.argv)) < 2:
      print "Usage: {0} <path_filename>".format(sys.argv[0])
      sys.exit(1)
  rospy.init_node("test_siar_path_tracker")
  frame_id = rospy.get_param('~frame_id', default="/base_link")    
  path_pub = rospy.Publisher('path',Path, queue_size=10, latch = True)
  # Test path
  vec = np.loadtxt(sys.argv[1])
  
  path = Path()
  path.header.frame_id = frame_id
  path.header.stamp = rospy.Time.now()
  path.header.seq = 0
  
  
  
  for i in range(len(vec)):
      row = vec[i]
      if len(row) >= 2:
            curr_wp = PoseStamped()
            curr_wp.header = path.header
            curr_wp.pose.orientation.x = 0
            curr_wp.pose.orientation.y = 0
            curr_wp.pose.orientation.z = 0
            curr_wp.pose.orientation.w = 1
            curr_wp.pose.position.z = 0
            curr_wp.pose.position.x = row[0]
            curr_wp.pose.position.y = row[1]
            path.poses.append(curr_wp)
  print "Path length: {0}".format(len(path.poses))
  path_pub.publish(path)
  
  # Spin until ctrl + c  
  rospy.spin()