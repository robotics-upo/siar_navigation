#ifndef _RRT_HPP_
#define _RRT_HPP_


#include "siar_planner/NodeState.hpp"
#include "siar_planner/RRTNode.h"
#include "siar_planner/SiarModel.hpp"

#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

#include <functions/functions.h>
#include <math.h>

#include <ros/ros.h>

class RRT
{
public:
  RRT(ros::NodeHandle &nh, ros::NodeHandle &pnh);
  
  ~RRT();
  
  double resolve(NodeState start, NodeState goal, std::list<RRTNode>& path);      
  double retCostPath(const std::list< RRTNode >& path);

  SiarModel &getModel() {return m;}

  std::list<RRTNode *> nodes;
  
//   visualization_msgs::Marker getPathMarker(const std::list< RRTNode >& path);
  visualization_msgs::MarkerArray getPathMarker(const std::list< RRTNode >& path);
  
  visualization_msgs::Marker getGraphMarker();
  
  double getDeltaT() const {return delta_t;}

  RRTNode goal_node;
  
protected:
  RRT();
  
  //int addNode(NodeState st, double comm_x = 0.0, double comm_ang = 0.0);
  void addNode(NodeState st, double comm_x = 0.0, double comm_ang = 0.0, RRTNode *parent = NULL); //darle uso a esta funcion
  void clear();
  
  void isGoal(NodeState st);
  bool got_to_goal = false;
  
  RRTNode start_node;
  
  //new functions
  NodeState getRandomState(double max_x, double max_y, double max_yaw, double min_x, double min_y, double min_yaw);
  RRTNode *getNearestNode(NodeState q_rand);
  void expandNode(const NodeState& q_rand, RRTNode* q_near, int relaxation_mode = 0);
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

RRT::~RRT()
{
  for (auto n:nodes) {
    delete n;
  }
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

NodeState RRT::getRandomState(double max_x, double min_x, double max_y, double min_y, double max_yaw, double min_yaw) {
  NodeState randomState;
  randomState.state.resize(3);
  randomState.state[0] = (dis(gen) * (max_x-min_x)) + min_x;  //set random value for each state varible
  randomState.state[1] = (dis(gen) * (max_y-min_y)) + min_y;
  randomState.state[2] = (dis(gen) * (max_yaw-min_yaw)) + min_yaw;
  
  return randomState;
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
//     std::cout << "El comando es " << command <<std::endl;
    double cost = m.integrate(st, command, delta_t, relaxation_mode >= 1); // If relaxation_mode >= 1 --> allow two wheels
    ROS_INFO("EL costo en rrt es: %f",cost);

    if (cost < 0.0) {
//       std::cout << "Colision " <<std::endl;
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


void RRT::isGoal(NodeState st) {
  got_to_goal = (fabs(st.state[0] - goal_node.st.state[0]) < goal_gap_m) && (fabs(st.state[1]-goal_node.st.state[1]) < goal_gap_m) &&
            (fabs(st.state[2] - goal_node.st.state[2]) < goal_gap_rad);
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


visualization_msgs::MarkerArray RRT::getPathMarker(const std::list< RRTNode >& path) 
{
  visualization_msgs::MarkerArray ret;
  visualization_msgs::Marker m_aux;

  int cont = 0;
  NodeState pt;
  for (auto it = path.begin(); it != path.end(); it++, cont++) {
    if (cont > 0) { 
      pt = (--it)->st;
      it++;
      geometry_msgs::Twist command;
      command.linear.x = it->command_lin;
      command.angular.z = it->command_ang;
      //  m_aux.action = visualization_msgs::Marker::DELETEALL;
      if (cont % 5 == 0) { 
        m_aux= m.getMarker(pt,cont);
        m_aux.color.b=1.0;
        m_aux.color.a=1.0;
        m_aux.color.g=0.2;
        m_aux.color.r=0.2;
        m_aux.lifetime = ros::Duration(2);
        ret.markers.push_back(m_aux);
      }
    }
  }
  return ret;
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

double RRT::retCostPath(const std::list< RRTNode >& path){
  double ret = 0;
  int cont = 0;

  for (auto it = path.begin(); it != path.end(); it++, cont++) {
    if (cont > 0) {
      double cost_node = fabs(it->cost);
      ret += cost_node;
    }
  }
  return ret;
}

#endif
