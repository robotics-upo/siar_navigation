#ifndef _RRT_HPP_
#define _RRT_HPP_


#include "siar_planner/NodeState.hpp"
#include "siar_planner/RRTNode.h"
#include "siar_planner/SiarModel.hpp"

#include "visualization_msgs/Marker.h"

#include <functions/functions.h>
#include <math.h>

#include <ros/ros.h>

class RRT
{
public:
  RRT(ros::NodeHandle &nh, ros::NodeHandle &pnh);
  double resolve(NodeState start, NodeState goal, std::list<RRTNode>& path);      
  
  SiarModel &getModel() {return m;}
  
  visualization_msgs::Marker getPathMarker(const std::list< RRTNode >& path);
  
  visualization_msgs::Marker getGraphMarker();
  
  double getDeltaT() const {return delta_t;}
  
protected:
  RRT();
  
  //int addNode(NodeState st, double comm_x = 0.0, double comm_ang = 0.0);
  void addNode(NodeState st, double comm_x = 0.0, double comm_ang = 0.0, RRTNode *parent = NULL); //darle uso a esta funcion
  void clear();
  
  std::list<RRTNode> nodes;
  
  void isGoal(NodeState st);
  bool got_to_goal = false;
  
  RRTNode goal_node;
  RRTNode start_node;
  
  
  //new functions
  NodeState getRandomState(double max_x, double max_y, double max_yaw, double min_x, double min_y, double min_yaw);
  RRTNode getNearestNode(NodeState q_rand);
  void expandNode(NodeState q_rand, RRTNode q_near, int relaxation_mode = 0);
  std::list<RRTNode> getPath();
//   NodeState euclideanDistance(NodeState q_rand, std::list<NodeState> list);
    
  // Sets to test
  bool file_test_set_init;
  std::vector <geometry_msgs::Twist> test_set_forward, test_set_backward;
  std::vector <geometry_msgs::Twist> file_test_set_forward, file_test_set_backward;  
  
  int K, n_iter, n_rounds;
  double delta_t, cellsize_m, cellsize_rad;
  double wheel_decrease, last_wheel;
  double max_x, max_y, max_yaw, min_x, min_y, min_yaw; //de que tipo serian?
  
  
  SiarModel m;
  double goal_gap_m, goal_gap_rad;
  
  int samp_cont, samp_goal_rate; //contadores de sampleo
  
  // Random numbers
  std::random_device rd;
  std::mt19937 gen;
  std::uniform_real_distribution<> dis;
};

RRT::RRT(ros::NodeHandle &nh, ros::NodeHandle &pnh):m(nh, pnh), gen(rd()), dis(0,1)
{
  if (!pnh.getParam("K", K)) { // Number of random commands
    K = 4;
  }
  
  if (!pnh.getParam("n_iter", n_iter)) {
    n_iter = 1000;
  }
  
  if (!pnh.getParam("n_rounds", n_rounds)) {
    n_rounds = 6;
  }
  
  if (!pnh.getParam("delta_t", delta_t)) {
    delta_t = 0.5;
  }
  
  if (!pnh.getParam("wheel_decrease", wheel_decrease)) {
    wheel_decrease = 0.05;
  }
  
  if (!pnh.getParam("last_wheel", last_wheel)) {
    last_wheel = 0.1;
  }
  
  if (!pnh.getParam("goal_gap_m", goal_gap_m)) {
    goal_gap_m = 0; //que pongo aqui
  }
  
  if (!pnh.getParam("goal_gap_rad", goal_gap_rad)) {
    goal_gap_rad = 0; //que pongo aqui
  }
  
  if (!pnh.getParam("max_x", max_x)) {  //change thsese default values 
    max_x = 1;
  }
  
  if (!pnh.getParam("max_y", max_y)) {
    max_y = 1;
  }
  
  if (!pnh.getParam("max_yaw", max_yaw)) {
    max_yaw = 1;
  }
  
  if (!pnh.getParam("min_x", min_x)) { //ver estos valores
    min_x = 0;
  }
  
  if (!pnh.getParam("min_y", min_y)) {
    min_y = 0;
  }
  
  if (!pnh.getParam("min_yaw", min_yaw)) {
    min_yaw = 0;
  }
  
  if (!pnh.getParam("samp_goal_rate", samp_goal_rate)) {
    samp_goal_rate = 10;
  }
  
  if (!pnh.getParam("cellsize_m", cellsize_m)) { //si quito esto, debo cambiar getGraphMarker
    cellsize_m = 0.2;
  }
  if (!pnh.getParam("cellsize_rad", cellsize_rad)) {
    cellsize_rad = 0.2;
  }
  
  ROS_INFO("n_iter = %d \t K: %d \t", n_iter, K); //cambiar esto
}


inline
void RRT::clear()
{
  nodes.clear();
}


double RRT::resolve(NodeState start, NodeState goal, std::list<RRTNode>& path)
{
  nodes.clear(); // Incremental algorithm --> the graph is generated in each calculation
  //path.clear(); //crearlo como variable de la clase??
  
  if (!m.isInit()) {
    ROS_ERROR("RRT::resolve --> The model has not been initialized --> could not calculate a path");
    return -1.0;
  }
  
  //samp_cont = 0;
  
  //RRTNode start_node; //modificar addnode para hacer esto
  start_node.st = start;
  nodes.push_back(start_node); 
  
  //RRTNode goal_node;
  goal_node.st = goal;
  //nodes.push_back(goal_node); //este nodo no se incluira en la lista
  
  double ret_val = -1.0; 
  int relax = 0;
  while (relax < n_rounds && !got_to_goal){
    int cont = 0; 
    while (cont < n_iter && !got_to_goal) { // n_iter Max. number of nodes to expand for each round
      //std::cout << "hasta aqui funciona inicio" << std::endl;
      NodeState q_rand;
      //if (!(samp_cont%samp_goal_rate == 0)){
      if (!(cont%samp_goal_rate == 0)){
	//std::cout << "hasta aqui funciona 1" << std::endl;
	q_rand = getRandomState(max_x, min_x, max_y, min_y, max_yaw, min_yaw);
      }
      else{
	q_rand = goal_node.st; 
	//std::cout << "hasta aqui funciona 2" << std::endl;
      }     
      RRTNode q_near = getNearestNode(q_rand);  
      expandNode(q_rand, q_near);
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
      ROS_ERROR("RRT::resolve -->  could not find a path -->  trying relaxation");    
    }
  }
  std::cout << "Numero de nodos en grafo: " << nodes.size() <<std::endl;
  return ret_val; 
}


inline
void RRT::addNode(NodeState st, double comm_x, double comm_ang, RRTNode *parent){ //si no le doy alguno de los elementos??
  RRTNode node;
  node.st = st;
  node.command_lin = comm_x;
  node.command_ang = comm_ang;
  node.parent = parent;
  nodes.push_back(node);
}

NodeState RRT::getRandomState(double max_x, double min_x, double max_y, double min_y, double max_yaw, double min_yaw) {
  NodeState randomState;
  randomState.state.resize(3);
  randomState.state[0] = (dis(gen) * (max_x-min_x)) + min_x;  //set random value for each state varible
  randomState.state[1] = (dis(gen) * (max_y-min_y)) + min_y;
  randomState.state[2] = (dis(gen) * (max_yaw-min_yaw)) + min_yaw;
  
  return randomState;
}

RRTNode RRT::getNearestNode(NodeState q_rand) {
  RRTNode q_near; 
  double dist = std::numeric_limits<double>::infinity(); 
  double new_dist;
//   TODO hacer una busqueda más eficiente
  for (auto it = nodes.begin()++ ; it != nodes.end(); ++it){ 
    new_dist = sqrt(pow(q_rand.state[0] - it->st.state[0],2) + pow(q_rand.state[1] - it->st.state[1],2)); 
    
    if (new_dist<dist) {
      q_near = *it; 
      dist = new_dist;
    }    
  }  
  return q_near;
}

void RRT::expandNode(NodeState q_rand, RRTNode q_near, int relaxation_mode){
  RRTNode q_new;
  double dist = std::numeric_limits<double>::infinity(); 
  double new_dist;
  bool new_node = false;
  
  //from q_near apply random conmands   
  NodeState st = q_near.st;
  for (int i = 0; i < K; i++) {
    geometry_msgs::Twist command = m.generateRandomCommand();
    double cost = m.integrate(st, command, delta_t, relaxation_mode >= 1); // If relaxation_mode >= 1 --> allow two wheels
    
    if (cost < 0.0) {
      // Collision
      // Update??
//       ROS_INFO("Detected collision. State: %s.\t Command: %f, %f", st.state.toString().c_str(), command.linear.x, command.angular.z);
    } 
    else {
      // get node with minimum distance
      new_node = true;
      new_dist = sqrt(pow(q_rand.state[0] - st.state[0],2) + pow(q_rand.state[1] - st.state[1],2));
      if (new_dist<dist) {
	q_new.st = st; //esto ha cambiado no? -> si, se pasa por referencia
	q_new.command_lin = command.linear.x;
	q_new.command_ang = command.angular.z;
	dist = new_dist;
      }
    }    
  }
  //check if there is a new node and add it to the graph (unless its the goal)
  if (new_node){
    //check if got to goal
    isGoal(q_new.st);
    //if(dist <= goal_gap_m){
    if(got_to_goal){
      q_near.children.push_back(&goal_node);
      goal_node.parent = &q_near; //como se pasa este valor
      goal_node.command_lin = q_new.command_lin; //?? esto es correcto? -> creo k si
      goal_node.command_ang = q_new.command_ang;
      got_to_goal = true;
      //nodes.push_back(goal_node);
    }
    else{
      q_near.children.push_back(&q_new);
      q_new.parent = &q_near; //como se pasa este valor??
      nodes.push_back(q_new);	
    }
  }
}

void RRT::isGoal(NodeState st) {
  got_to_goal = (fabs(st.state[0] - goal_node.st.state[0]) < goal_gap_m) && (fabs(st.state[1]-goal_node.st.state[0]) < goal_gap_m) &&
            (fabs(st.state[2] - goal_node.st.state[0]) < goal_gap_rad);
}

//std::list<RRTNode> RRT::getPath(std::list<RRTNode> nodes){ //hace falta pasarle argumento?, siendo una variable global
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

visualization_msgs::Marker RRT::getPathMarker(const std::list< RRTNode >& path) 
{
  visualization_msgs::Marker ret;
  
  int cont = 0;
  NodeState st;
  for (auto it = path.begin(); it != path.end(); it++, cont++) {
    if (cont > 0) { //cuando no lo es??
      st = (--it)->st;
      it++;
    
      geometry_msgs::Twist command;
      command.linear.x = it->command_lin;
      command.angular.z = it->command_ang;
    
      m.integrate(ret, st, command, 1.0, true);
    }
  }
  
  return ret;
}

visualization_msgs::Marker RRT::getGraphMarker()
{
  visualization_msgs::Marker m;
  m.header.frame_id = this->m.getFrameID();
  m.header.stamp = ros::Time::now();
  m.ns = "rrt";
  m.action = visualization_msgs::Marker::ADD;
  m.pose.orientation.w = 1.0;
  m.id = 0;
  m.points.clear();
  //m.type = visualization_msgs::Marker::POINTS;
  m.type = visualization_msgs::Marker::LINE_LIST;
  // LINE_LIST markers use x scale only (for line width)
  m.scale.x = cellsize_m / 3.0;
  // Points are green
  visualization_msgs::Marker::_color_type color;
  color.r = 1.0;
  color.b = 0;
  color.g = 0;
  color.a = 1.0;
  double color_step = 1.0/(double)nodes.size();
  geometry_msgs::Point p1;
  geometry_msgs::Point p2;
  //for (unsigned int i = 0; i < nodes.size();i++) {
  for (auto it = nodes.begin(); it != nodes.end(); it++){  
    
    auto new_color = color;
    new_color.r -= color_step;
    new_color.b += color_step;
    
    p1.x = it->st.state[0];
    p1.y = it->st.state[1];
    
    m.points.push_back(p1); //es correcto aqui?
    m.colors.push_back(color);
    
     for (auto it2 = it ->children.begin(); it2 != it -> children.end(); ++it2) {
       m.points.push_back(p1);
       m.colors.push_back(color);
       
       p2.x = (*it2)->st.state[0];
       p2.y = (*it2)->st.state[1];
       
       m.points.push_back(p2);
       m.colors.push_back(new_color);
     }
   
    color = new_color;
    
  }
  
  return m;
}

#endif
