#ifndef _tRRT_HPP_
#define _tRRT_HPP_


#include "siar_planner/NodeState.hpp"
#include "siar_planner/RRTNode.h"
#include "siar_planner/SiarModel.hpp"

#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

#include <functions/functions.h>
#include <math.h>

#include <ros/ros.h>

class tRRT
{
public:
  tRRT(ros::NodeHandle &nh, ros::NodeHandle &pnh);
  
  ~tRRT();
  
  double resolve(NodeState start, NodeState goal, std::list<RRTNode>& path);    
  double retCostPath(const std::list< RRTNode >& path);  
  
  SiarModel &getModel() {return m;}
  
  visualization_msgs::MarkerArray getPathMarker(const std::list< RRTNode >& path);
  visualization_msgs::Marker getGraphMarker();
  
  double getDeltaT() const {return delta_t;}
  std::list<RRTNode *> nodes;

  RRTNode goal_node;

protected:
  tRRT();
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

  //bool transitionTest (NodeState &q_near, NodeState &q_new, NodeState& q_rand, double cost, double parent_cost, double dist);
  bool transitionTest (NodeState q_near, NodeState q_new, NodeState q_rand);
  void calculateCost(NodeState q_near, NodeState q_new, NodeState q_rand,double &cost_Qnear,double &cost_Qnew);
  
  double K_normal = 1000000; //(cost_initial - cost_goal)/2
  double Temp = 1*exp (-6); //* exp(-3);
  int alfa = 2, nFail = 0, nFailmax = 2;
  double cost_Qnear, cost_Qnew, cost_wheels_Qnew;
    
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

tRRT::tRRT(ros::NodeHandle &nh, ros::NodeHandle &pnh):m(nh, pnh), gen(rd()), dis(0,1)
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
    goal_gap_m = 0; 
  }  
  if (!pnh.getParam("goal_gap_rad", goal_gap_rad)) {
    goal_gap_rad = 0; 
  }
  if (!pnh.getParam("max_x", max_x)) {  
    max_x = 1;
  }
  if (!pnh.getParam("max_y", max_y)) {
    max_y = 1;
  }
  if (!pnh.getParam("max_yaw", max_yaw)) {
    max_yaw = 1;
  }
  if (!pnh.getParam("min_x", min_x)) { 
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
  if (!pnh.getParam("cellsize_m", cellsize_m)) { 
    cellsize_m = 0.2;
  }
  if (!pnh.getParam("cellsize_rad", cellsize_rad)) {
    cellsize_rad = 0.2;
  }
  ROS_INFO("n_iter = %d \t K: %d \t", n_iter, K); 
}

tRRT::~tRRT()
{
  for (auto n:nodes) {
    delete n;
  }
}


inline void tRRT::clear()
{
  nodes.clear();
}


double tRRT::resolve(NodeState start, NodeState goal, std::list<RRTNode>& path)
{
  nodes.clear(); 

  if (!m.isInit()) {
    ROS_ERROR("tRRT::resolve --> The model has not been initialized --> could not calculate a path");
    return -1.0;
  }

  start_node.st = start;
  nodes.push_back(new RRTNode(start_node)); 
  goal_node.st = goal;
  
  double ret_val = -1.0; 
  int relax = 0;
  
  while (relax < n_rounds && !got_to_goal){ //n_round=6
    int cont = 0; 
    while (cont < n_iter && !got_to_goal) { // n_iter = 200 Max. number of nodes to expand for each round
      // ROS_ERROR("Iniciando   la     busqueda    de     nodos     en iteracion de RELAX: %i /%i, e iteracion de CONT: %i /%i", relax+1,n_rounds, cont+1,n_iter);
      // ROS_INFO("El tamano de la lista con nodos es: %ld",nodes.size());
      // for (auto n: nodes){ 
      //   ROS_INFO("El valor del NODO en la lista es es X = %f, Y = %f , th = %f",n->st.state[0],n->st.state[1],n->st.state[2]);  
      // }
      NodeState q_rand;
      if (!(cont%samp_goal_rate == 0)){
	      q_rand = getRandomState(max_x, min_x, max_y, min_y, max_yaw, min_yaw);
      }
      else{
      	q_rand = goal_node.st; 
      }     
      // ROS_INFO("El nodo q_rand elegido en GETRANDOMSTATE es X = %f, Y = %f , th = %f", q_rand.state[0],q_rand.state[1],q_rand.state[2]);  
      RRTNode *q_near = getNearestNode(q_rand);  
      // ROS_INFO("El nodo q_near elegido en getNearestNode es X = %f, Y = %f , th = %f", q_near->st.state[0],q_near->st.state[1],q_near->st.state[2]); 
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


NodeState tRRT::getRandomState(double max_x, double min_x, double max_y, double min_y, double max_yaw, double min_yaw) {
  NodeState randomState;
  randomState.state.resize(3);
  randomState.state[0] = (dis(gen) * (max_x-min_x)) + min_x;  //set random value for each state varible
  randomState.state[1] = (dis(gen) * (max_y-min_y)) + min_y;
  randomState.state[2] = (dis(gen) * (max_yaw-min_yaw)) + min_yaw;
  return randomState;
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

    NodeState st_rand =  q_rand;    // just to normalize the names of the nodes, this case Qrand
    NodeState st_near = q_near->st; // is necesary to keep one value of Qnear to apply the transition

    geometry_msgs::Twist command = m.generateRandomCommand(); //generate a random command of velocity
    // ROS_ERROR("Evaluando Iteracion: %i / %i para diferentes velocidades", i+1 , K);
    // ROS_INFO("El NODO st utilizado en funcion INTEGRATE es X = %f, Y = %f , th = %f", st.state[0],st.state[1],st.state[2]);
    cost_wheels_Qnew = m.integrateTransition(st, command, delta_t);
    // ROS_INFO("EL costo en trrt es: %f",cost_wheels_Qnew); 
    if (cost_wheels_Qnew < 0.0)
      continue;

    // ROS_INFO("El NODO q_new proveniente de INTEGRATE que va a entrar a TRANSITIONTEST es X = %f, Y = %f , th = %f", st.state[0],st.state[1],st.state[2]);
    ret_transition = transitionTest (st_near, st, st_rand);  //Get the value of Q_new from st of integrate function if there is not collision
    // ROS_INFO("TransitionTest retorna un %d",ret_transition);

    if (ret_transition){
      is_new_node = true;
      new_dist = sqrt(pow(q_rand.state[0] - st.state[0],2) + pow(q_rand.state[1] - st.state[1],2));
      // ROS_INFO("El NODO q_new evaluado para agregar a la LISTA es X = %f, Y = %f , th = %f. ¿es  new_dist(%f)  < dist(%f) ?", st.state[0],st.state[1],st.state[2],new_dist,dist);
      if (new_dist<dist) {
        q_new.st = st; 
        q_new.command_lin = command.linear.x;
        q_new.command_ang = command.angular.z;
        // ROS_INFO("EL NODO q_new ha sido ACEPTADO para agregarce a la lista, los valore sson X=%f, Y=%f, th=%f",q_new.st.state[0],q_new.st.state[1],q_new.st.state[2] );
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
      goal_node.cost = q_new.cost + q_near->cost; // New field:cost
    }
    else{
      q_new.parent = q_near;
      q_new.cost += q_near->cost; // Accumulate the parent cost 
      RRTNode *new_node = new RRTNode(q_new); 
      q_near->children.push_back(new_node);
      nodes.push_back(new_node);	
    }
  }
}


bool tRRT::transitionTest (NodeState q_near, NodeState q_new, NodeState q_rand){ // Retorna un true o false diciendo si se acepta o no la configuracion

  // ROS_ERROR ("We ARE inside of the TRANSITION");
  double transition_probability;
  
  std::uniform_real_distribution<> dis_2(0,1);
  double random_prob = dis_2(gen);
  double cost_max = 10000000000;
  calculateCost(q_near, q_new, q_rand, cost_Qnear, cost_Qnew);
  // ROS_INFO("El NODO q_new dentro de TRANSITIONTEST despues de CALCULATECOST es X = %f, Y = %f , th = %f", q_new.state[0],q_new.state[1],q_new.state[2]);
  double dist_Qnear_Qnew = sqrt(pow(q_near.state[0] - q_new.state[0],2) + pow(q_near.state[1] - q_new.state[1],2));;
  double slope_cost = (cost_Qnew - cost_Qnear)/dist_Qnear_Qnew;
  //  ROS_INFO ("The value distance_Qnear_Qnew es: %f", dist_Qnear_Qnew);

  if (slope_cost > 0){
    transition_probability = exp (-(slope_cost)/(K_normal * Temp)); 
  }
  else {
    transition_probability = 1;
  }
  if (cost_Qnew > cost_max){
    //  ROS_ERROR("Can't be apply the transitionTest --> The cost_Qnew of new_node exceeds the maximum cost");
    return 0;
  }
  if(cost_Qnew < cost_Qnear ){ 
    //  ROS_ERROR("cost_Qnew < cost_Qnear, por lo tanto transition probability = 1, se acepta inmediatamente la configuracion");   
    return 1;
  }
  if (random_prob < transition_probability){
    Temp = Temp/ alfa;
    nFail = 0;
    //  ROS_INFO(" random_prob es MENOR que transition_probability. El valor de nFail es: %i",nFail);
    //  ROS_INFO ("El valor de slope_cost es %f, de Temperature es: %f. El random_prob es: %f y la probabilidad de transicion: %f",slope_cost,Temp, random_prob,transition_probability);
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
    //  ROS_INFO(" random_prob es MAYOR que transition_probability. El valor de nFail es: %i",nFail);
    //  ROS_INFO ("El valor de slope_cost es %f, de Temperature es: %f. El random_prob es: %f y la probabilidad de transicion: %f",slope_cost,Temp, random_prob,transition_probability);
    return 0;
  }
}


void tRRT::calculateCost(NodeState q_near, NodeState q_new, NodeState q_rand,double &cost_Qnear,double &cost_Qnew){
  //First calculate the Cost of Qnear
  double cost_dis_Qnear = sqrt(pow(q_rand.state[0] - q_near.state[0],2) + pow(q_rand.state[1] - q_near.state[1],2));
  double cost_wheels_Qnear = m.costWheelsQnear(q_near.state[0],q_near.state[1],q_near.state[2]);

  //Second calculate the Cost of Qnew
  double cost_dist_Qnew = sqrt(pow(q_rand.state[0] - q_new.state[0],2) + pow(q_rand.state[1] - q_new.state[1],2));
  
  cost_Qnear = cost_dis_Qnear*1.5 + cost_wheels_Qnear;
  cost_Qnew = cost_dist_Qnew*1.5  + cost_wheels_Qnew;
  // ROS_INFO("El NODO q_new en CALCULATECOST es X = %f, Y = %f , th = %f", q_new.state[0],q_new.state[1],q_new.state[2]);
  // ROS_INFO("El NODO q_near en CALCULATECOST es X = %f, Y = %f , th = %f", q_near.state[0],q_near.state[1],q_near.state[2]);
  // ROS_INFO("El NODO q_rand en CALCULATECOST es X = %f, Y = %f , th = %f", q_rand.state[0],q_rand.state[1],q_rand.state[2]);
  // ROS_INFO("El costo Qnear es: %f (dis= %f, wheels=%f) y Qnew es: %f (dis= %f, wheels=%f)",cost_Qnear,cost_dis_Qnear,cost_wheels_Qnear, cost_Qnew,cost_dist_Qnew,cost_wheels_Qnew);
}


void tRRT::isGoal(NodeState st) {
  got_to_goal = (fabs(st.state[0] - goal_node.st.state[0]) < goal_gap_m) && (fabs(st.state[1]-goal_node.st.state[1]) < goal_gap_m) &&
            (fabs(st.state[2] - goal_node.st.state[2]) < goal_gap_rad);
}


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

visualization_msgs::MarkerArray tRRT::getPathMarker(const std::list< RRTNode >& path) 
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


double tRRT::retCostPath(const std::list< RRTNode >& path){
  double ret;
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

