#ifndef _tRRT_HPP_
#define _tRRT_HPP_

#include "siar_planner/Planner.hpp"

#include "siar_planner/NodeState.hpp"
#include "siar_planner/RRTNode.h"
#include "siar_planner/SiarModel.hpp"
#include "siar_planner/Transition.hpp"

#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

#include <functions/functions.h>
#include <math.h>

#include <ros/ros.h>

class tRRT:public Planner
{
public:
  tRRT(ros::NodeHandle &nh, ros::NodeHandle &pnh);
  
  ~tRRT();
  
  virtual double resolve(NodeState start, NodeState goal, std::list<RRTNode>& path);    
  
  virtual visualization_msgs::Marker getGraphMarker();
  
  virtual int getGraphSize() {
    return nodes.size();
  }

  std::list<RRTNode *> nodes;

protected:
  // tRRT();
  void addNode(NodeState st, double comm_x = 0.0, double comm_ang = 0.0, RRTNode *parent = NULL); 
  virtual void clear();
  
  //new functions
  RRTNode *getNearestNode(NodeState q_rand);
  void expandNode(const NodeState& q_rand, RRTNode* q_near, int relaxation_mode = 0);
  virtual std::list<RRTNode> getPath();

  //bool transitionTest (NodeState &q_near, NodeState &q_new, NodeState& q_rand, double cost, double parent_cost, double dist);
  // bool transitionTest (NodeState q_near, NodeState q_new, NodeState q_rand);
  // void calculateCost(NodeState q_near, NodeState q_new, NodeState q_rand,double &cost_Qnear,double &cost_Qnew);
  
  Transition testTrans;

  double K_normal, Temp_init, Temp; //(cost_initial - cost_goal)/2
  int alfa , nFail, nFailmax ;
  double cost_Qnear, cost_Qnew, cost_wheels_Qnew;


};

tRRT::tRRT(ros::NodeHandle &nh, ros::NodeHandle &pnh):Planner(nh,pnh),testTrans(nh,pnh)
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

  // ROS_INFO("n_iter = %d \t K: %d \t", n_iter, K); 
}

tRRT::~tRRT()
{
  clear();
}


inline void tRRT::clear()
{
  for (auto n:nodes) {
    delete n;
  }
  nodes.clear();
  got_to_goal = false;
}


double tRRT::resolve(NodeState start, NodeState goal, std::list<RRTNode>& path)
{
  clear(); 

  if (!m.isInit()) {
    ROS_ERROR("tRRT::resolve --> The model has not been initialized --> could not calculate a path");
    return -1.0;
  }
  
  
  RRTNode start_node;
  geometry_msgs::Twist command_init=m.generateRandomCommand(); 

  start_node.st = start;
  start_node.cost = m.integrateTransition(start_node.st, command_init, delta_t);
  nodes.push_back(new RRTNode(start_node)); 
  goal_node.st = goal;

  
  double ret_val = -1.0; 
  int relax = 0;
  Temp = Temp_init;
  while (relax < n_rounds && !got_to_goal){ //n_round=6
    int cont = 0; 
    while (cont < n_iter && !got_to_goal) { // n_iter = 200 Max. number of nodes to expand for each round
    
      NodeState q_rand;
      if (!(cont%samp_goal_rate == 0)){
	      q_rand = getRandomState(max_x, min_x, max_y, min_y, max_yaw, min_yaw);
      }
      else{
      	q_rand = goal_node.st; 
      }     
      RRTNode *q_near = getNearestNode(q_rand);  
      expandNode(q_rand, q_near, relax);
      cont++;
    }
    
    if(got_to_goal){ 
      path = getPath(); 
      ret_val = 1;
      ROS_INFO("Iteration %d. Solution found", relax);
    }
    else{   //if didnt get a solution, do relaxation
      m.decreaseWheels(wheel_decrease, last_wheel);
      relax++;
      ROS_ERROR("tRRT::resolve -->  could not find a path -->  starting new iteration");    
    }
  }
  std::cout << "Numero de nodos en grafo: " << nodes.size() <<std::endl;
  return ret_val; 
}

RRTNode* tRRT::getNearestNode(NodeState q_rand) {
  RRTNode *q_near = NULL; 
  double dist = std::numeric_limits<double>::infinity(); 
  double new_dist;
  for (auto n: nodes){ 
    new_dist = sqrt(pow(q_rand.state[0] - n->st.state[0],2) + pow(q_rand.state[1] - n->st.state[1],2)); 
    if (new_dist < dist) {
      q_near = n; 
      dist = new_dist;
    }    
  }
  return q_near;
}


void tRRT::expandNode(const NodeState &q_rand, RRTNode *q_near, int relaxation_mode){
  RRTNode q_new;
  double dist = std::numeric_limits<double>::infinity(); 
  double new_dist;
  bool is_new_node = false;
  bool ret_transition = false; 
  for (int i = 0; i < K; i++) {   //Are generate K different nodes (4) with different STATES to evaluate !!
    ret_transition = false;
    is_new_node = false;
    NodeState st = q_near->st;
    geometry_msgs::Twist command = m.generateRandomCommand(); //generate a random command of velocity
    cost_wheels_Qnew = m.integrateTransition(st, command, delta_t);
    // ROS_INFO("value cost_wheels_Qnew %f",cost_wheels_Qnew);
    RRTNode q_try = q_new;
    q_try.st = st;
    q_try.cost = cost_wheels_Qnew;
    if (cost_wheels_Qnew < 0.0)
      continue;
      ret_transition = testTrans.transitionTest(*q_near, q_try);
// ROS_INFO("Value Transition: %d",ret_transition);
    // ret_transition = transitionTest (st_near, st, st_rand);  //Get the value of Q_new from st of integrate function if there is not collision
    if (ret_transition){
      is_new_node = true;
      new_dist = sqrt(pow(q_rand.state[0] - st.state[0],2) + pow(q_rand.state[1] - st.state[1],2));
      if (new_dist<dist) {
        q_new.st = st; 
        q_new.command_lin = command.linear.x;
        q_new.command_ang = command.angular.z;
        dist = new_dist;
        q_new.cost = cost_wheels_Qnew; // New field:cost        
      }
    } 
  }
  if (is_new_node){   //check if there is a new node and add it to the graph (unless its the goal)
     isGoal(q_new.st);
     if(got_to_goal){
      q_near->children.push_back(&goal_node);
      goal_node.parent = q_near; 
      goal_node.command_lin = q_new.command_lin; 
      goal_node.command_ang = q_new.command_ang;  
      // goal_node.cost = q_new.cost + q_near->cost; // New field:cost
    }
    else{
      q_new.parent = q_near;
      // q_new.cost += q_near->cost; // Accumulate the parent cost 
      RRTNode *new_node = new RRTNode(q_new); 
      q_near->children.push_back(new_node);
      nodes.push_back(new_node);	
    }
  }
}



// bool tRRT::transitionTest (NodeState q_near, NodeState q_new, NodeState q_rand){ // Retorna un true o false diciendo si se acepta o no la configuracion

//   double transition_probability;
  
//   std::uniform_real_distribution<> dis_2(0,1);
//   double random_prob = dis_2(gen);
//   double cost_max = 10000000000;
//   calculateCost(q_near, q_new, q_rand, cost_Qnear, cost_Qnew);
//   double dist_Qnear_Qnew = sqrt(pow(q_near.state[0] - q_new.state[0],2) + pow(q_near.state[1] - q_new.state[1],2));;
//   double slope_cost = (cost_Qnew - cost_Qnear)/dist_Qnear_Qnew;
//   if (slope_cost > 0){
//     transition_probability = exp (-(slope_cost)/(K_normal * Temp)); 
//   }
//   else {
//     transition_probability = 1;
//   }
//   if (cost_Qnew > cost_max){
//     return 0;
//   }
//   if(cost_Qnew < cost_Qnear ){ 
//     return 1;
//   }
//   if (random_prob < transition_probability){
//     Temp = Temp/ alfa;
//     nFail = 0;
//     return 1;
//   }
//   else{
//     if (nFail > nFailmax){  // Set in 0 and 20 respectively
//       Temp = Temp * 10 *alfa;
//       nFail = 0;  
//     }
//     else
//     {
//       nFail = nFail+1 ; 
//     }
//     return 0;
//   }
// }


// void tRRT::calculateCost(NodeState q_near, NodeState q_new, NodeState q_rand,double &cost_Qnear,double &cost_Qnew){
//   //First calculate the Cost of Qnear
//   double cost_dis_Qnear = sqrt(pow(q_rand.state[0] - q_near.state[0],2) + pow(q_rand.state[1] - q_near.state[1],2));
//   double cost_wheels_Qnear = m.costWheelsQnear(q_near.state[0],q_near.state[1],q_near.state[2]);

//   //Second calculate the Cost of Qnew
//   double cost_dist_Qnew = sqrt(pow(q_rand.state[0] - q_new.state[0],2) + pow(q_rand.state[1] - q_new.state[1],2));
  
//   cost_Qnear = cost_dis_Qnear*1.5 + cost_wheels_Qnear;
//   cost_Qnew = cost_dist_Qnew*1.5  + cost_wheels_Qnew;
// }


std::list<RRTNode> tRRT::getPath(){
  //get path from finished graph
  std::list<RRTNode> path;
  RRTNode* current_node = &goal_node;
  path.push_front(*current_node);
  while (current_node->parent != NULL) {  
    current_node = current_node->parent;
    path.push_front(*current_node);
  }
  return path;
}

visualization_msgs::Marker tRRT::getGraphMarker()
{
  visualization_msgs::Marker m;
  m.header.frame_id = this->m.getFrameID();
  m.header.stamp = ros::Time::now();
  m.pose.orientation.w = 1.0;
  m.id = 0;
  m.points.clear();
  m.type = visualization_msgs::Marker::POINTS;
  m.scale.x = 0.05;
  visualization_msgs::Marker::_color_type color;
  color.r = 1.0;
  color.b = 0;
  color.g = 0;
  color.a = 1.0;
  double color_step = 1.0/(double)nodes.size();
  geometry_msgs::Point p1;
  geometry_msgs::Point p2;
  for (auto n : nodes){  
    auto new_color = color;
    new_color.r -= color_step;
    new_color.b += color_step;
    p1.x = n->st.state[0];
    p1.y = n->st.state[1];
    m.points.push_back(p1); //es correcto aqui?
    m.colors.push_back(color);
    color = new_color;
  }
  m.lifetime = ros::Duration(2);

  return m;
}

#endif

