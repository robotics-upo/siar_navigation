#ifndef _RRT_HPP_
#define _RRT_HPP_

#include "siar_planner/Planner.hpp"
#include "siar_planner/NodeState.hpp"
#include "siar_planner/RRTNode.h"
#include "siar_planner/SiarModel.hpp"

#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

#include <functions/functions.h>
#include <math.h>

#include <ros/ros.h>

class RRT:public Planner
{
public:
  RRT(ros::NodeHandle &nh, ros::NodeHandle &pnh);
  
  ~RRT();
  
  virtual double resolve(NodeState start, NodeState goal, std::list<RRTNode>& path);      

  std::list<RRTNode *> nodes; // TODO: single tree planners
  
//   visualization_msgs::Marker getPathMarker(const std::list< RRTNode >& path);
  virtual visualization_msgs::Marker getGraphMarker();

  virtual int getGraphSize() {
    return nodes.size();
  }
  
protected:
  // RRT();
  
  //int addNode(NodeState st, double comm_x = 0.0, double comm_ang = 0.0);
  void addNode(NodeState st, double comm_x = 0.0, double comm_ang = 0.0, RRTNode *parent = NULL); //darle uso a esta funcion
  virtual void clear();
  
  //new functions
  virtual RRTNode *getNearestNode(NodeState q_rand);
  void expandNode(const NodeState& q_rand, RRTNode* q_near, int relaxation_mode = 0);
  virtual std::list<RRTNode> getPath();
//   NodeState euclideanDistance(NodeState q_rand, std::list<NodeState> list);
    
};

RRT::RRT(ros::NodeHandle &nh, ros::NodeHandle &pnh):Planner(nh,pnh)
{
  
  
  ROS_INFO("n_iter = %d \t K: %d \t", n_iter, K); //cambiar esto
}

RRT::~RRT()
{
  clear();
  
}


inline
void RRT::clear()
{
  for (auto n:nodes) {
    delete n;
  }
  nodes.clear();
  got_to_goal = false;
}


double RRT::resolve(NodeState start, NodeState goal, std::list<RRTNode>& path)
{
  clear(); // Incremental algorithm --> the graph is generated in each calculation
  //path.clear(); //crearlo como variable de la clase??
  
  if (!m.isInit()) {
    ROS_ERROR("RRT::resolve --> The model has not been initialized --> could not calculate a path");
    return -1.0;
  }
  
  //samp_cont = 0;
  
  //RRTNode start_node; //modificar addnode para hacer esto
  start_node.st = start;
  nodes.push_back(new RRTNode(start_node)); 
  
  //RRTNode goal_node;
  goal_node.st = goal;
  //nodes.push_back(goal_node); //este nodo no se incluira en la lista
  
  double ret_val = -1.0; 
  int relax = 0;
  
  while (relax < n_rounds && !got_to_goal){
    int cont = 0; 
    while (cont < n_iter && !got_to_goal) { // n_iter Max. number of nodes to expand for each round
      NodeState q_rand;
      //if (!(samp_cont%samp_goal_rate == 0)){
      if (!(cont%samp_goal_rate == 0)){
	      q_rand = getRandomState(max_x, min_x, max_y, min_y, max_yaw, min_yaw);
      }
      else{
	      q_rand = goal_node.st; 
      }     
      RRTNode *q_near = getNearestNode(q_rand);  
      expandNode(q_rand, q_near, relax);
//       expandNode(q_rand, q_near);
      cont++;
      //samp_cont++;
    }
    
    if(got_to_goal){ 
      //if got solution, may return path
      //std::list<RRTNode> path;
      path = getPath(); 
      ret_val = 1;
      ROS_INFO("Iteration %d. Solution found", relax);
      //create markers for path
      //getPathMarker(path);      
    }
    else{ 
      //if didnt get a solution, do relaxation
      m.decreaseWheels(wheel_decrease, last_wheel);
      relax++;
      ROS_ERROR("RRT::resolve -->  could not find a path -->  starting new iteration");    
    }
  }
  std::cout << "Numero de nodos en grafo: " << nodes.size() <<std::endl;
  return ret_val; 
}

RRTNode* RRT::getNearestNode(NodeState q_rand) {
  RRTNode *q_near = NULL; 
  double dist = std::numeric_limits<double>::infinity(); 
  double new_dist;
//   TODO hacer una busqueda más eficiente
  for (auto n: nodes){ 
    new_dist = sqrt(pow(q_rand.state[0] - n->st.state[0],2) + pow(q_rand.state[1] - n->st.state[1],2)); 
    
    if (new_dist < dist) {
      q_near = n; 
      dist = new_dist;
    }    
  }  
  return q_near;
}

void RRT::expandNode(const NodeState &q_rand, RRTNode *q_near, int relaxation_mode){
  RRTNode q_new;
  double dist = std::numeric_limits<double>::infinity(); 
  double new_dist;
  bool is_new_node = false;
  
  //from q_near apply random conmands   
  
  for (int i = 0; i < K; i++) {
    NodeState st = q_near->st;
    geometry_msgs::Twist command = m.generateRandomCommand();
    // std::cout << "El comando es " << command <<std::endl;
    double cost = m.integrate(st, command, delta_t, relaxation_mode >= 1); // If relaxation_mode >= 1 --> allow two wheels
    // ROS_INFO("EL costo en rrt es: %f",cost);

    if (cost < 0.0) {
      // Collision
      // Update??
//       ROS_INFO("Detected collision. State: %s.\t Command: %f, %f", st.state.toString().c_str(), command.linear.x, command.angular.z);
    } 
    else {
//       std::cout << "Se encuentra nodo sin colision " <<std::endl;
      // get node with minimum distance
      is_new_node = true;
      new_dist = sqrt(pow(q_rand.state[0] - st.state[0],2) + pow(q_rand.state[1] - st.state[1],2));
      if (new_dist<dist) {
      q_new.st = st; 
      q_new.command_lin = command.linear.x;
      q_new.command_ang = command.angular.z;
      dist = new_dist;
      q_new.cost = cost; // New field:cost
      }
    }    
  }
  //check if there is a new node and add it to the graph (unless its the goal)
  if (is_new_node){
    //check if got to goal
    isGoal(q_new.st);
    //if(dist <= goal_gap_m){
    if(got_to_goal){
      q_near->children.push_back(&goal_node);
      goal_node.parent = q_near; 
      goal_node.command_lin = q_new.command_lin; 
      goal_node.command_ang = q_new.command_ang;
      goal_node.cost = q_new.cost + q_near->cost; // New field:cost
    }
    else{
      q_new.parent = q_near;
      q_new.cost += q_near-> cost; // Accumulate the parent cost 
      RRTNode *new_node = new RRTNode(q_new); 
      q_near->children.push_back(new_node);
      nodes.push_back(new_node);	
    }
  }
}


std::list<RRTNode> RRT::getPath(){
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

visualization_msgs::Marker RRT::getGraphMarker()
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
