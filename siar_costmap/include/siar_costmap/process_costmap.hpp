#ifndef __PROCESSCOSTMAP_HPP__
#define __PROCESSCOSTMAP_HPP__

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ANN/ANN.h>
#include <iostream>
#include <siar_costmap/siarcostmap.hpp>

class ProcessCostmap
{
  struct PointPix { int x,y;};
public:

  // Class constructor
    ProcessCostmap()
    {
        // Load parameters
        ros::NodeHandle nh;
        ros::NodeHandle lnh("~");
        if(!lnh.getParam("exp_decay", m_expDecay))
            m_expDecay = 0.0;

        if (!lnh.getParam("positive_obstacle", m_positiveObs))
            m_positiveObs = SC_POSITIVE_OBS;
        if (!lnh.getParam("negative_obstacle", m_negativeObs))
            m_negativeObs = SC_NEGATIVE_OBS;
        m_costmapPub = nh.advertise<nav_msgs::OccupancyGrid>("/processed_costmap", 1);
        m_costmapSub = nh.subscribe("/costmap", 2, &ProcessCostmap::costmapReceived, this);
    }

  void costmapReceived(const nav_msgs::OccupancyGridConstPtr &orig_costmap) {
    nav_msgs::OccupancyGrid m_costmap = *orig_costmap;
    std::vector<struct PointPix> obstacles;
    int k = 0;
    for(int i=0; i < m_costmap.info.height; i++)
      {
        for(int j=0; j < m_costmap.info.width; j++, k++)
        {
            if(m_costmap.data[k] == m_positiveObs || m_costmap.data[k] == m_negativeObs) {
                struct PointPix a;
                a.x = i;
                a.y = j;
                obstacles.push_back(a);
            }
        }
      }
    ROS_INFO("ProcessCostmap: obstacles detected: %lu", obstacles.size());
    // Build the object positions array
    if ( m_expDecay > 0.0 && obstacles.size() > 0) {
      ANNpointArray obsPts = annAllocPts(obstacles.size(), 2);
      // Build the Kdtree structure for effitient search
    //   ANNkd_tree* kdTree = new ANNkd_tree(obsPts, obstacles.size(), 2);
    ANNbruteForce* kdTree = new ANNbruteForce(obsPts, obstacles.size(), 2);

      ANNpoint queryPt = annAllocPt(2);
      ANNidxArray nnIdx = new ANNidx[1];
      ANNdistArray dists = new ANNdist[1];
      float C = log(127.0/0.1)/m_expDecay;
      for(int i=0; i < obstacles.size(); i++)
      {
        obsPts[i][0] = obstacles[i].x;
        obsPts[i][1] = obstacles[i].y;
      }
      ROS_INFO("ProcessCostmap: performing decay");

      for(int i=0, k=0; i<m_costmap.info.height; i++)
      {
          ROS_INFO("Row: %d", i);
        for(int j=0; j<m_costmap.info.width; j++, k++)
        {
            if(m_costmap.data[k] != m_positiveObs && m_costmap.data[k] != m_negativeObs) {
                // Compute distance to closest obstacle
                queryPt[0] = i;
                queryPt[1] = j;
                kdTree->annkSearch(queryPt, 1, nnIdx, dists);
                // Evaluate the cost

                m_costmap.data[k] = (int)(100.0*exp(-C*sqrt(dists[0])));
                // std::cout << "Writing: " << (int)m_costmap.data[k] << "\t" << dists[0] << "\n";
            }
        }
      }
    }
    m_costmapPub.publish(m_costmap);
  }

private:

  // Params
  float m_expDecay;
  int m_positiveObs, m_negativeObs;

  // Comms
  ros::Publisher m_costmapPub;
  ros::Subscriber m_costmapSub;
};


#endif

