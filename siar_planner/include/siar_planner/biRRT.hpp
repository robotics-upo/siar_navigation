#ifndef _biRRT_HPP_
#define _biRRT_HPP_


#include "siar_planner/NodeState.hpp"
#include "siar_planner/RRTNode.h"
#include "siar_planner/SiarModel.hpp"

#include "visualization_msgs/Marker.h"

#include <functions/functions.h>
#include <math.h>

#include <ros/ros.h>

class biRRT
{
public:
  biRRT(ros::NodeHandle &nh, ros::NodeHandle &pnh);
  
  ~biRRT();
  
  double resolve(NodeState start, NodeState goal, std::list<RRTNode>& path);      
  
  SiarModel &getModel() {return m;}
  
//   visualization_msgs::Marker getPathMarker(const std::list< RRTNode >& path);
  visualization_msgs::Marker getPathMarker(const std::list< RRTNode >& path);
  
  visualization_msgs::Marker getGraphMarker();
  
  double getDeltaT() const {return delta_t;}
  
protected:
  biRRT();
  
  void clear();
  
  std::list<RRTNode *> tree1;
  std::list<RRTNode *> tree2;
  
  RRTNode* areConnected(NodeState st, bool direct);
  bool got_connected = false;
  
  RRTNode goal_node;
  RRTNode start_node;
  
  
  //new functions
  NodeState getRandomState(double max_x, double max_y, double max_yaw, double min_x, double min_y, double min_yaw);
  RRTNode *getNearestNode(NodeState q_rand, bool primary_tree);
  void expandNode(const NodeState& q_rand, RRTNode* q_near, int relaxation_mode = 0, bool direct = true);
  std::list<RRTNode> getPath();
  void expandNearestNodes();
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

biRRT::biRRT(ros::NodeHandle &nh, ros::NodeHandle &pnh):m(nh, pnh), gen(rd()), dis(0,1)
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

biRRT::~biRRT()
{
  for (auto n:tree1) {
    delete n;
  }
  for (auto n:tree2) {
    delete n;
  }
}


inline
void biRRT::clear()
{
  tree1.clear();
  tree2.clear();
}


double biRRT::resolve(NodeState start, NodeState goal, std::list<RRTNode>& path)
{
  
  tree1.clear(); // Incremental algorithm --> the graph is generated in each calculation
  tree2.clear();
  path.clear(); //crearlo como variable de la clase??
  
  if (!m.isInit()) {
    ROS_ERROR("biRRT::resolve --> The model has not been initialized --> could not calculate a path");
    return -1.0;
  }
  
  
  //RRTNode start_node; //modificar addnode para hacer esto
  start_node.st = start;
  tree1.push_back(new RRTNode(start_node)); 
  
  //RRTNode goal_node;
  goal_node.st = goal;
  tree2.push_back(new RRTNode(goal_node));
  //nodes.push_back(goal_node); //este nodo no se incluira en la lista
  
  double ret_val = -1.0; 
  int relax = 0;
  
  while (relax < n_rounds && !got_connected){
    int cont = 0; 
    while (cont < n_iter && !got_connected) { // n_iter Max. number of nodes to expand for each round
      NodeState q_rand;
      if (!(cont%samp_goal_rate == 0)){
	q_rand = getRandomState(max_x, min_x, max_y, min_y, max_yaw, min_yaw);
	RRTNode *q_near = getNearestNode(q_rand, true);  
        expandNode(q_rand, q_near, relax, true);
	q_near = getNearestNode(q_rand, false);
	expandNode(q_rand, q_near, relax, false);
      }
      else{
// 	expandNearestNodes();
	double dist = std::numeric_limits<double>::infinity();
	RRTNode *q_closest1 = NULL;  
	RRTNode *q_closest2 = NULL;
	double new_dist;
	for (auto n1: tree1){ 
	  for (auto n2: tree2){
	    new_dist = sqrt(pow(n1->st.state[0] - n2->st.state[0],2) + pow(n1->st.state[1] - n2->st.state[1],2)); 
	    if (new_dist < dist){
	      q_closest1 = n1; 
	      q_closest2 = n2; 
	      dist = new_dist;
	    }
	  }
	}
	//aqui llamo a expandNode
	expandNode(q_closest2->st, q_closest1, relax, true); //expansion de q_closest1 que pertenece a tree1 hacia tree2
	expandNode(q_closest1->st, q_closest2, relax, false); //expansion de q_closest2 que pertenece a tree2 hacia tree1	
      }     
      cont++;
    }
    
    if(got_connected){ 
      //if got solution, may return path
      path = getPath(); 
      ret_val = 1;
      ROS_INFO("Iteration %d. Solution found", relax);
    }
    else{ 
      //if didnt get a solution, do relaxation
      m.decreaseWheels(wheel_decrease, last_wheel);
      relax++;
      std::cout << "Numero de nodos en grafo: tree1 : " << tree1.size() << "  tree2: " <<tree2.size() <<std::endl;
      ROS_ERROR("biRRT::resolve -->  could not find a path -->  trying relaxation");    
    }
  }
  std::cout << "Numero de nodos en grafo: " << tree1.size()+tree2.size() <<std::endl;
  return ret_val; 
}


NodeState biRRT::getRandomState(double max_x, double min_x, double max_y, double min_y, double max_yaw, double min_yaw) {
  NodeState randomState;
  randomState.state.resize(3);
  randomState.state[0] = (dis(gen) * (max_x-min_x)) + min_x;  //set random value for each state varible
  randomState.state[1] = (dis(gen) * (max_y-min_y)) + min_y;
  randomState.state[2] = (dis(gen) * (max_yaw-min_yaw)) + min_yaw;
  
  return randomState;
}

RRTNode* biRRT::getNearestNode(NodeState q_rand, bool primary_tree) {
  RRTNode *q_near = NULL; 
  double dist = std::numeric_limits<double>::infinity(); 
  double new_dist;
//   TODO hacer una busqueda más eficiente
  if (primary_tree){
    for (auto n: tree1){ 
      new_dist = sqrt(pow(q_rand.state[0] - n->st.state[0],2) + pow(q_rand.state[1] - n->st.state[1],2)); 
      if (new_dist < dist) {
	q_near = n; 
        dist = new_dist;
      }
    } 
  }
  else{
    for (auto n: tree2){ 
      new_dist = sqrt(pow(q_rand.state[0] - n->st.state[0],2) + pow(q_rand.state[1] - n->st.state[1],2)); 
      if (new_dist < dist) {
	q_near = n; 
        dist = new_dist;
      }
    } 
  } 
  return q_near;
}

void biRRT::expandNode(const NodeState &q_rand, RRTNode *q_near, int relaxation_mode, bool direct){
  RRTNode q_new;
  double dist = std::numeric_limits<double>::infinity(); 
  double new_dist;
  bool is_new_node = false;
  
  //from q_near apply random conmands   
  
  for (int i = 0; i < K; i++) {
    NodeState st = q_near->st;
    geometry_msgs::Twist command = m.generateRandomCommand();
//     std::cout << "El comando es " << command <<std::endl;
    double cost;
    if (direct){
      cost = m.integrate(st, command, delta_t, relaxation_mode >= 1); // If relaxation_mode >= 1 --> allow two wheels
      
      if (cost < 0.0) {
// 	std::cout << "Colision " <<std::endl;
      }
      else{
// 	std::cout << "Se encuentra nodo sin colision " <<std::endl;
	// get node with minimum distance
	is_new_node = true;
	new_dist = sqrt(pow(q_rand.state[0] - st.state[0],2) + pow(q_rand.state[1] - st.state[1],2));
	if (new_dist<dist) {
	  q_new.st = st; 
	  q_new.command_lin = command.linear.x;
	  q_new.command_ang = command.angular.z;
	  dist = new_dist; 
	}
      }
    }
    
    else{
      cost = m.integrate(st, command, -(delta_t), relaxation_mode >= 1); // si direct es false, entonces le paso delta_t <0 a la integracion
      if (cost < 0.0) {
// 	std::cout << "Colision " <<std::endl;
      }
      else{
// 	std::cout << "Se encuentra nodo sin colision " <<std::endl;
	// get node with minimum distance
	is_new_node = true;
	new_dist = sqrt(pow(q_rand.state[0] - st.state[0],2) + pow(q_rand.state[1] - st.state[1],2));
	if (new_dist<dist) {
	  q_new.st = st; 
	  q_near->command_lin = command.linear.x; //este dato se almacena en el hijo
	  q_near->command_ang = command.angular.z;
	  dist = new_dist;
	}
      }
    }  
  }
  //check if there is a new node and add it to the graph (unless its the goal)
  if (is_new_node){
    //check if got to goal
    RRTNode *q_closest = areConnected(q_new.st, direct); //este q_closest solo lo uso si got_connected=true; si es direct, q_closest sera un hijo, si no, sera un padre
      
    //esto tambien depende, si es direct o no
    if(got_connected){
      if(direct){
	q_near->children.push_back(q_closest); //considero q_closest casi igual a q_new
	q_closest->parent = q_near; 
	q_closest->command_lin = q_new.command_lin; 
	q_closest->command_ang = q_new.command_ang;
      }
      else{
	q_closest->children.push_back(q_near); //considero q_closest casi igual a q_new
	q_near->parent = q_closest; 
	q_near->command_lin = q_new.command_lin; 
	q_near->command_ang = q_new.command_ang;
      }      
    }
    else{
      if(direct){
	q_new.parent = q_near;
	RRTNode *new_node = new RRTNode(q_new); //para pasar puntero?? no se puede pasar el puntero como argumento??
	q_near->children.push_back(new_node);
	tree1.push_back(new_node);
      }
      else{
// 	std::cout << "hay un nodo indirecto " <<std::endl;
	q_near->parent = &q_new;
	q_near->command_lin = q_new.command_lin; //esto debe recibirlo el hijo, que en este caso es q_near
	q_near->command_ang = q_new.command_ang;
	q_new.command_lin = 0;
	q_new.command_ang = 0;
	RRTNode *new_node = new RRTNode(q_new); 
	new_node->children.push_back(q_near);
	tree2.push_back(new_node);
      }
    }
  }
}

RRTNode* biRRT::areConnected(NodeState st, bool direct) {
  double dist = std::numeric_limits<double>::infinity();
  RRTNode *q_closest = NULL;  
  double new_dist;
  if(direct){
    for (auto n: tree2){ //si hago integracion positiva obtengo nodo de tree1, debo compraralo con tree2
      new_dist = sqrt(pow(st.state[0] - n->st.state[0],2) + pow(st.state[1] - n->st.state[1],2)); 
      if (new_dist < dist) {
	q_closest = n; 
        dist = new_dist;
      }
    }  
  }
  else{
    for (auto n: tree1){ //si hago integracion positiva obtengo nodo de tree1, debo compraralo con tree2
      new_dist = sqrt(pow(st.state[0] - n->st.state[0],2) + pow(st.state[1] - n->st.state[1],2)); 
      if (new_dist < dist) {
	q_closest = n; 
        dist = new_dist;
      }
    }
  }
  got_connected = (fabs(st.state[0] - q_closest->st.state[0]) < goal_gap_m) && (fabs(st.state[1]-q_closest->st.state[1]) < goal_gap_m) && (fabs(st.state[2] - q_closest->st.state[2]) < goal_gap_rad);
  return q_closest;	    
}


std::list<RRTNode> biRRT::getPath(){
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

// void biRRT::expandNearestNodes(){
//   double dist = std::numeric_limits<double>::infinity();
//   RRTNode *q_closest1 = NULL;  
//   RRTNode *q_closest2 = NULL;
//   double new_dist;
//   for (auto n1: tree1){ 
//     for (auto n2: tree2){
//       new_dist = sqrt(pow(n1->st.state[0] - n2->st.state[0],2) + pow(n1->st.state[1] - n2->st.state[1],2)); 
//       if (new_dist < dist){
// 	q_closest1 = n1; 
// 	q_closest2 = n2; 
//         dist = new_dist;
//       }
//     }
//   }
//   //aqui llamo a expandNode
//   expandNode(q_closest2->st, q_closest1, relax, true); //expansion de tree1
//   expandNode(q_closest1->st, q_closest2, relax, false); //expansion de tree2
//     
// }

// visualization_msgs::Marker biRRT::getPathMarker(const std::list< RRTNode >& path) 
// {
//   visualization_msgs::Marker ret;
//   
//   int cont = 0;
//   NodeState st;
//   for (auto it = path.begin(); it != path.end(); it++, cont++) {
//     if (cont > 0) { //cuando no lo es??
//       st = (--it)->st;
//       it++;
//     
//       geometry_msgs::Twist command;
//       command.linear.x = it->command_lin;
//       command.angular.z = it->command_ang;
//     
//       m.integrate(ret, st, command, 1.0, true);
//     }
//   }
//   
//   return ret;
// }

visualization_msgs::Marker biRRT::getPathMarker(const std::list< RRTNode >& path)
{
  visualization_msgs::Marker m;
  m.header.frame_id = this->m.getFrameID();
  m.header.stamp = ros::Time::now();
  m.ns = "path";
  m.action = visualization_msgs::Marker::ADD;
  m.pose.orientation.w = 1.0;
  m.id = 0;
  m.points.clear();
  m.type = visualization_msgs::Marker::POINTS;
//   m.type = visualization_msgs::Marker::LINE_LIST;
  // LINE_LIST markers use x scale only (for line width)
  m.scale.x = 0.05;
  // Points are green
  visualization_msgs::Marker::_color_type color;
  color.r = 0;
  color.b = 0;
  color.g = 1.0;
  color.a = 1.0;
//   double color_step = 1.0/(double)nodes.size();
  geometry_msgs::Point p1;
  //geometry_msgs::Point p2;
  //for (unsigned int i = 0; i < nodes.size();i++) {
//   for (auto n : path){  
  for (auto it = path.begin(); it != path.end(); it++){
//     auto new_color = color;
//     new_color.r -= color_step;
//     new_color.b += color_step;
    
    p1.x = it->st.state[0];
    p1.y = it->st.state[1];
    
    m.points.push_back(p1); //es correcto aqui?
    m.colors.push_back(color);
    
//      for (auto child : n->children) {
//        m.points.push_back(p1);
//        m.colors.push_back(color);
// //        
//        p2.x = child->st.state[0];
//        p2.y = child->st.state[1];
// //        
//        m.points.push_back(p2);
//        m.colors.push_back(new_color);
//      }
   
//     color = new_color;
    
  }
  
  return m;
}

visualization_msgs::Marker biRRT::getGraphMarker()
{
  visualization_msgs::Marker m;
  m.header.frame_id = this->m.getFrameID();
  m.header.stamp = ros::Time::now();
  m.ns = "rrt";
  m.action = visualization_msgs::Marker::ADD;
  m.pose.orientation.w = 1.0;
  m.id = 0;
  m.points.clear();
  m.type = visualization_msgs::Marker::POINTS;
//   m.type = visualization_msgs::Marker::LINE_LIST;
  // LINE_LIST markers use x scale only (for line width)
  m.scale.x = 0.05;
  // Points are green
  visualization_msgs::Marker::_color_type color;
  color.g = 0;
  color.a = 1.0;
//   double color_step = 1.0/(double)nodes.size();
  geometry_msgs::Point p1;
//   geometry_msgs::Point p2;
  //for (unsigned int i = 0; i < nodes.size();i++) {
  for (auto n : tree1){  
    color.r = 0;
    color.b = 1.0;
//     auto new_color = color;
//     new_color.r -= color_step;
//     new_color.b += color_step;
    
    p1.x = n->st.state[0];
    p1.y = n->st.state[1];
    
    m.points.push_back(p1); 
    m.colors.push_back(color);
  }
  
  for (auto n : tree2){  
    color.r = 1.0;
    color.b = 0;
//     auto new_color = color;
//     new_color.r -= color_step;
//     new_color.b += color_step;
    
    p1.x = n->st.state[0];
    p1.y = n->st.state[1];
    
    m.points.push_back(p1); 
    m.colors.push_back(color);   
  }
  
  return m;
}

#endif
