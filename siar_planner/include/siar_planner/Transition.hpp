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

  double alfa , nFail, nFailmax ,K_normal, Temp_init, Temp; //(cost_initial - cost_goal)/2
  double cost_Qnear, cost_Qnew,Cost_max_transition;

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
    nFailmax = 4;
  }
  if (!pnh.getParam("Temp_init",Temp_init)) {
    Temp_init = 1e-6;
  }
  if (!pnh.getParam("Cost_max_transition",Cost_max_transition)) {
    Cost_max_transition = 25000;
  }
  Temp=Temp_init; 
  ROS_INFO("K_normal = %f ;  alfa: %f;  nFailmax: %f;  Temp_init: %f;  Cost_max_transition: %f \t", K_normal,alfa,nFailmax,Temp_init,Cost_max_transition); 
}


bool Transition::transitionTest (RRTNode q_near, RRTNode q_new){ 

  double transition_probability;  
  std::uniform_real_distribution<> dis_2(0,1);
  double random_prob = dis_2(gen);
  cost_Qnew = q_new.cost;
  cost_Qnear = q_near.cost;

  // ROS_INFO("q_new: x = %f, y= %f   -   q_near: x = %f, y = %f",q_new.st.state[0],q_new.st.state[1],q_near.st.state[0],q_near.st.state[1]);
  double dist_Qnear_Qnew = sqrt(pow(q_near.st.state[0] - q_new.st.state[0],2) + pow(q_near.st.state[1] - q_new.st.state[1],2));;
  double slope_cost = (cost_Qnew - cost_Qnear)/dist_Qnear_Qnew;
  // std::cout << "cost_Qnew: " << cost_Qnew << ";  cost_Qnear: " << cost_Qnear <<";  distancia: "  << dist_Qnear_Qnew << std::endl;
  // std::cout << "slope cost: " << slope_cost << ";  Temp: " << Temp << ";  nFail: " << nFail << ";  K_normal: "<< K_normal << ";  alfa: "<<alfa << std::endl;
  // std::cout << "transition_probability: " << transition_probability << ";  random_prob: " << random_prob << std::endl;
  
  //Calculating Transition Probability, referred as the Bolzmann probibility
  if (slope_cost > 0){
    transition_probability = exp (-(slope_cost)/(K_normal * Temp)); 
  }
  else {
    transition_probability = 1;
  }
  // std::cout << "transition_probability: " << transition_probability << ";  random_prob: " << random_prob << std::endl;
  // Evaluating the Transition Test
  if (cost_Qnew > Cost_max_transition){
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
    if (nFail > nFailmax){  
      Temp = Temp *alfa;
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

