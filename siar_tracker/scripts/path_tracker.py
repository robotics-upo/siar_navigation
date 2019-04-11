#!/usr/bin/env python

PKG = 'siar_tracker'
import roslib; roslib.load_manifest(PKG)
import rospy
import tf
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist
import numpy as np
import sys
import time
from threading import Thread, Lock
import math

class PathTracker:
  def path_callback(self, new_path):

      self.mutex.acquire()
      self.path = Path()
      self.path.header.stamp = new_path.header.stamp
      self.path.header.seq = new_path.header.seq
      self.path.header.frame_id = self.global_frame_id

      self.path_len = len(new_path.poses)

      print "New path received. Frame id: {0}. Global frame id: {1}".format( new_path.header.frame_id, self.global_frame_id)

      for i in range(len( new_path.poses )):
	if new_path.header.frame_id != self.global_frame_id:
            try:
                print "Waypoint {0}".format(i)
                new_pose = self.listener.transformPose(self.global_frame_id, new_path.poses[i])
                self.path.poses.append(new_pose)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
	            print "SiarPathTracker error: could not get transform between {0} and {1}".format(self.global_frame_id, new_path.poses[i].header.frame_id)
	else:
	  self.path.poses.append(new_path.poses[i]);

      print "New path received. Length: {0}. New path length: {1}". format(len(new_path.poses), len(self.path.poses))
      self.active_path = True
      self.curr_wp = 0
      self.mutex.release()

  def __init__(self):
    # Default parameters
    self.base_frame_id = rospy.get_param('~base_frame_id', default="/base_link")
    self.global_frame_id = rospy.get_param('~global_frame_id', default="/world")
    self.goal_gap = rospy.get_param('~goal_gap',default=0.2)
    self.v = rospy.get_param('~v', default=0.2)
    self.a = rospy.get_param('~a', default=0.2)
    self.lookahead = rospy.get_param('~lookahead', default=2)
    # Flags and internal stuff
    self.active_path = False # Indicates if we have received a path request
    self.mutex = Lock()
    # Set up your subscriber and define its callback
    rospy.Subscriber("path", Path, self.path_callback)
    # Setup publisher
    self.cmd_vel_pub = rospy.Publisher('cmd_vel',Twist, queue_size=2)
    self.listener = tf.TransformListener()

  def loop_function(self):
    rate = rospy.Rate(10.0)
    dist = [0,0]
    while not rospy.is_shutdown():
        cmd = Twist()
        cmd.linear.x = 0
        cmd.angular.z = 0
        if self.active_path and len(self.path.poses) > 0:
            self.mutex.acquire()
            try:
                (trans,rot) = self.listener.lookupTransform(self.global_frame_id, self.base_frame_id, rospy.Time(0))
                # Search for the closest node
                min_dist = 1e100
                new_min = False
                for i in range(self.curr_wp, len(self.path.poses)):
                    p = self.path.poses[i]
                    dist[0] = trans[0] - p.pose.position.x
                    dist[1] = trans[1] - p.pose.position.y
                    d = dist[0]**2 + dist[1]**2
                    print "d = {0} min_dist = {1}". format(d, min_dist)
                    if d < min_dist:
                        new_min = True
                        min_dist = d
                        self.curr_wp = i
                    else:
                        break # If the distance increases, we found the closest waypoint

                #Get the new target taking into account the lookahead
                target_node = min(self.curr_wp + self.lookahead, len(self.path.poses) - 1)
                
                p = self.path.poses[target_node]
                dist[0] = trans[0] - p.pose.position.x
                dist[1] = trans[1] - p.pose.position.y
                d = math.sqrt(dist[0]**2 + dist[1]**2)
                print "Target node {0}, length:{1}, dist: {2}".format(target_node, len(self.path.poses), d)
                if d < self.goal_gap and target_node + 1 >= len(self.path.poses):
                    self.active_path = False
                    print "Goal reached!!"
                else:
                    cmd.linear.x = self.v
                    cmd.angular.z = self.a * math.atan2(dist[1], dist[0])
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                print "Path tracker --> Could not get the transform"
            self.mutex.release()
        self.cmd_vel_pub.publish(cmd)
        rate.sleep()

if __name__ == '__main__':
  rospy.init_node("siar_path_tracker")
  tracker = PathTracker()
  print "Tracker created"

  #Generate the path follower thread
  t = Thread(target=tracker.loop_function)
  t.daemon = True
  t.start()

  # Spin until ctrl + c

  rospy.spin()

