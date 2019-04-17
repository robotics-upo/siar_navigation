#ifndef _TRANSITION_HPP_
#define _TRANSITION_HPP_

#include "siar_planner/Planner.hpp"
#include "siar_planner/NodeState.hpp"
#include "siar_planner/RRTNode.h"
#include "siar_planner/SiarModel.hpp"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

#include <functions/functions.h>
#include <math.h>
#include <ros/ros.h>

class Transition
{
public:
  Transition(ros::NodeHandle &nh, ros::NodeHandle &pnh); 
//   ~Transition();
    
    bool transitionTest (RRTNode q_near, RRTNode q_new);
    // void calculateCost(NodeState q_near, NodeState q_new, double &cost_Qnear,double &cost_Qnew);

protected:

  double K_normal, Temp_init, Temp; //(cost_initial - cost_goal)/2
  int alfa , nFail, nFailmax ;
  double cost_Qnear, cost_Qnew;

  std::mt19937 gen;

  
};

Transition::Transition(ros::NodeHandle &nh, ros::NodeHandle &pnh)
{
  if (!pnh.getParam("K_normal", K_normal)) {
    K_normal = 1000000;
  }
  if (!pnh.getParam("alfa", alfa)) { 
    alfa = 2;
  }
  if (!pnh.getParam("nFailmax", nFailmax)) {
    nFailmax = 0.2;
  }
  if (!pnh.getParam("Temp_init",Temp_init)) {
    Temp_init = 1e-6;
  }
//   ROS_INFO("K_normal = %d \t alfa: %d \t nFailmax: %d \t Temp_init: %d \t", K_normal,alfa,nFailmax,Temp_init); 
}


bool Transition::transitionTest (RRTNode q_near, RRTNode q_new){ // Retorna un true o false diciendo si se acepta o no la configuracion

  double transition_probability;  
  std::uniform_real_distribution<> dis_2(0,1);
  double random_prob = dis_2(gen);
  double cost_max = 10000000000;
  cost_Qnew = q_new.cost;
  cost_Qnear = q_near.cost;

    // ROS_INFO("Value cost_Qnew: %f  -  cost_Qnear: %f",cost_Qnew,cost_Qnear);
  double dist_Qnear_Qnew = sqrt(pow(q_near.st.state[0] - q_new.st.state[0],2) + pow(q_near.st.state[1] - q_new.st.state[1],2));;
  double slope_cost = (cost_Qnew - cost_Qnear)/dist_Qnear_Qnew;


  if (slope_cost > 0){
    transition_probability = exp (-(slope_cost)/(K_normal * Temp)); 
  }
  else {
    transition_probability = 1;
  }
  if (cost_Qnew > cost_max){
    return 0;
  }
  if(cost_Qnew < cost_Qnear ){ 
    return 1;
  }
  if (random_prob < transition_probability){
    Temp = Temp/ alfa;
    nFail = 0;
    return 1;
  }
  else{
    if (nFail > nFailmax){  // Set in 0 and 20 respectively
      Temp = Temp * 10 *alfa;
      nFail = 0;  
    }
    else
    {
      nFail = nFail+1 ; 
    }
    return 0;
  }
}

#endif

