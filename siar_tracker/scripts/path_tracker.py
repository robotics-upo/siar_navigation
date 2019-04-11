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

class PathTracker:    
  def path_callback(self, new_path):
      self.mutex.acquire()
      self.path = Path()
      self.path.poses
      self.path.header = new_path.header
      self.path.header.frame_id = global_frame_id
      
      self.path_len = len(self.path.poses)
      print ("New path received. Length: {0}". format(self.path_len))
            
      # TODO: Transform the path to the base frame
      if (path.header.frame_id != self.global_frame_id)
        for i in range(len( new_path.poses ))
            try
                new_pose = self.listener.transformPose(global_frame_id, new_path.poses[i])
                self.path.poses.append(new_pose)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
      self.active_path = True
      self.curr_wp = 0
      self.mutex.release()
      
  def __init__():
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
    
  def loop_function:
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        cmd = Twist()
        cmd.linear.x = 0
        cmd.angular.z = 0
        if (self.active_path):
            self.mutex.acquire()
            try:
                (trans,rot) = listener.lookupTransform(self.global_frame_id, self.base_frame_id, rospy.Time(0))
                # Search for the closest node
                min_dist = 1e100
                new_min = False
                for i in range(self.curr_wp, len(self.path.poses)):
                    p = self.path.poses[i]
                    dist[0] = trans[0] - p.pose.position.x
                    dist[1] = trans[1] - p.pose.position.y
                    d = dist[0]**2 + dist[1]**2
                    if d < min_dist:
                        new_min = True
                        min_dist = d
                    else:
                        if new_min = False:
                            self.curr_wp = i - 1
                            break # If the distance increases, we found the current waypoint
                        
                #Get the new target taking into account the lookahead
                target_node = min(self.curr_wp + self.lookahead, len(self.path.poses))
                p = self.path.target_node.poses[target_node]
                dist[0] = trans[0] - p.pose.position.x
                dist[1] = trans[1] - p.pose.position.y
                d = dist[0]**2 + dist[1]**2
                if min_dist < self.goal_gap and self.curr_wp + 1 >= len(self.path.poses):
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
  PathTracker tracker()
  
  # Spin until ctrl + c  
  t = Thread(target=tracker.loop_function)
  t.daemon = True
  t.start()  
  rospy.spin()