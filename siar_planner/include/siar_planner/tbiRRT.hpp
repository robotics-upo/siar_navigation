#ifndef _tbiRRT_HPP_
#define _tbiRRT_HPP_

#include "siar_planner/NodeState.hpp"
#include "siar_planner/RRTNode.h"
#include "siar_planner/SiarModel.hpp"

#include "visualization_msgs/Marker.h"
#include <visualization_msgs/MarkerArray.h>
#include <functions/functions.h>
#include <math.h>

#include <ros/ros.h>

class tbiRRT
{
public:
  tbiRRT(ros::NodeHandle &nh, ros::NodeHandle &pnh);
  
  ~tbiRRT();
  
  double resolve(NodeState start, NodeState goal, std::list<RRTNode>& path); 
  double resolve_expand1(NodeState start, NodeState goal, std::list<RRTNode>& path);
  double retCostPath(const std::list< RRTNode >& path);  
  
  SiarModel &getModel() {return m;}
  
  visualization_msgs::MarkerArray getPathMarker(const std::list< RRTNode >& path);
  visualization_msgs::Marker getGraphMarker();
  
  double getDeltaT() const {return delta_t;}
  
  std::list<RRTNode *> tree1;
  std::list<RRTNode *> tree2;
  RRTNode *q_final_1 = NULL;
  RRTNode *q_final_2 = NULL;
  
protected:
  tbiRRT();
  
  void clear();
  
  RRTNode* areConnected(NodeState st, bool direct);
  bool got_connected = false;
  
  RRTNode goal_node;
  RRTNode start_node;
   
  //new functions
  NodeState getRandomState(double max_x, double max_y, double max_yaw, double min_x, double min_y, double min_yaw);
  RRTNode *getNearestNode(NodeState q_rand, bool primary_tree);
  RRTNode *getNearestNode1(NodeState q_rand);
  void expandNode1(const NodeState& q_rand, RRTNode* q_near, int relaxation_mode = 0, bool direct = true);
  void expandNode2(const NodeState& q_rand, RRTNode* q_near, int relaxation_mode = 0, bool direct = true);
  std::list<RRTNode> getPath();
  void expandNearestNodes();
  bool transitionTest1 (NodeState q_near, NodeState q_new, NodeState q_rand);
  bool transitionTest2 (NodeState q_near, NodeState q_new, NodeState q_rand);
  // void calculateCost(NodeState q_near, NodeState q_new, NodeState q_rand,double &cost_Qnear,double &cost_Qnew);
  double calculateCostNear1(NodeState q_near, NodeState q_new, NodeState q_rand);
  double calculateCostNew1(NodeState q_near, NodeState q_new, NodeState q_rand, double cost_wheels_Qnew);
  double calculateCostNear2(NodeState q_near, NodeState q_new, NodeState q_rand);
  double calculateCostNew2(NodeState q_near, NodeState q_new, NodeState q_rand, double cost_wheels_Qnew);
  double K_normal = 1000000; //(cost_initial - cost_goal)/2
  double Temp = 1*exp (-6); //* exp(-3);
  int alfa = 2, nFail = 0, nFailmax = 4;
  double Temp1 = 1*exp (-6); //* exp(-3);
  int nFail1 = 0, nFailmax1 = 4;
  double cost_wheels_Qnew1, cost_wheels_Qnew2;
    
  // Sets to test
  bool file_test_set_init;
  std::vector <geometry_msgs::Twist> test_set_forward, test_set_backward;
  std::vector <geometry_msgs::Twist> file_test_set_forward, file_test_set_backward;  
  
  int K, n_iter, n_rounds;
  double delta_t, cellsize_m, cellsize_rad;
  double wheel_decrease, last_wheel;
  double max_x, max_y, max_yaw, min_x, min_y, min_yaw; //de que tipo serian?
  double x_g, y_g, x_0, y_0;
  
  SiarModel m;
  double goal_gap_m, goal_gap_rad;
  int samp_cont, samp_goal_rate; //contadores de sampleo
  bool tree_num;
  
  // Random numbers
  std::random_device rd;
  std::mt19937 gen;
  std::uniform_real_distribution<> dis;
};

tbiRRT::tbiRRT(ros::NodeHandle &nh, ros::NodeHandle &pnh):m(nh, pnh), gen(rd()), dis(0,1)
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
  if (!pnh.getParam("max_x", max_x)) {  //change these default values 
    max_x = 1;
  }
  if (!pnh.getParam("max_y", max_y)) {
    max_y = 1;
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
  if (!pnh.getParam("max_yaw", max_yaw)) {
    max_yaw = 1;
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

tbiRRT::~tbiRRT(){
  for (auto n:tree1) {
    delete n;
  }
  for (auto n:tree2) {
    delete n;
  }
}


inline void tbiRRT::clear()
{
  tree1.clear();
  tree2.clear();
  q_final_1 = NULL;
  q_final_2 = NULL;
  got_connected = false;
}


double tbiRRT::resolve(NodeState start, NodeState goal, std::list<RRTNode>& path){
  clear();
  
  if (!m.isInit()) {
    ROS_ERROR("tbiRRT::resolve --> The model has not been initialized --> could not calculate a path");
    return -1.0;
  }

  start_node.st = start;
  tree1.push_back(new RRTNode(start_node)); 
  goal_node.st = goal;
  tree2.push_back(new RRTNode(goal_node));
  
  double ret_val = -1.0; 
  int relax = 0;
  
  while (relax < n_rounds && !got_connected) {

    int cont = 0; 
    while (cont < n_iter && !got_connected) { // n_iter Max. number of nodes to expand for each round
      // ROS_ERROR("Comenzo el ciclo while en resolve, relax/n_rounds = %i /%i",relax,n_rounds);
      // ROS_ERROR("Comenzo el ciclo while en resolve, cont/n_inter = %i /%i",cont,n_iter);
      NodeState q_rand;
      if (!(cont%samp_goal_rate == 0)){
	      q_rand = getRandomState(max_x, min_x, max_y, min_y, max_yaw, min_yaw);
        // ROS_INFO("ARBOL 1");
        // ROS_INFO("q_rand es X = %f, Y = %f , th = %f", q_rand.state[0],q_rand.state[1],q_rand.state[2]);  
	      RRTNode *q_near = getNearestNode(q_rand, true);  //This function only allow to select the q_near for one of the two tree with the BOOL value (this case tree from start)
        // ROS_INFO("q_near es X = %f, Y = %f , th = %f", q_near->st.state[0],q_near->st.state[1],q_near->st.state[2]); 
        expandNode1(q_rand, q_near, relax, true);         //The BOOL value allow to change de simbol (+ o -) of m_delta_T to change the direction of the velocity in the evaluateTrajectory (here +)
	      if(!got_connected){                              //got_cnected is TRU if the to tree connected
	        q_near = getNearestNode(q_rand, false);        //This function only allow to select the q_near for one of the two tree with the BOOL value (this case tree from goal)
          // ROS_INFO("ARBOL 2");
          // ROS_INFO("q_rand es X = %f, Y = %f , th = %f", q_rand.state[0],q_rand.state[1],q_rand.state[2]); 
          // ROS_INFO("q_near es X = %f, Y = %f , th = %f", q_near->st.state[0],q_near->st.state[1],q_near->st.state[2]); 
          expandNode2(q_rand, q_near, relax, false);      //The BOOL value allow to change de simbol (+ o -) of m_delta_T to change the direction of the velocity in the evaluateTrajectory (here -)

        }
      }
      else{                                               // is search the nodes from each tree that are more near
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
        //  ROS_INFO("El nodo n_1 es X = %f, Y = %f , th = %f", q_closest1->st.state[0],q_closest1->st.state[1],q_closest1->st.state[2]);  
        //  ROS_INFO("El nodo n_2 es X = %f, Y = %f , th = %f", q_closest2->st.state[0],q_closest2->st.state[1],q_closest2->st.state[2]);  
        //aqui llamo a expandNode
        expandNode1(q_closest2->st, q_closest1, relax, true); //expansion de q_closest1 que pertenece a tree1 hacia tree2
        if(!got_connected){
          expandNode2(q_closest1->st, q_closest2, relax, false); //expansion de q_closest2 que pertenece a tree2 hacia tree1
        }
      }     
      cont++;
    }
    
    if(got_connected){ 
      path = getPath(); 
      ret_val = 1;
      ROS_INFO("Iteration %d. Solution found", relax);    
    }
    else{ 
      m.decreaseWheels(wheel_decrease, last_wheel);
      relax++;
      ROS_ERROR("tbiRRT::resolve -->  could not find a path -->  starting new iteration");    
    }
  }
  std::cout << "Numero de nodos totales: " << tree1.size()+tree2.size() <<std::endl;
  std::cout << "Numero de nodos en path: " << path.size() <<std::endl;
  return ret_val; 
}


NodeState tbiRRT::getRandomState(double max_x, double min_x, double max_y, double min_y, double max_yaw, double min_yaw) {
  NodeState randomState;
  randomState.state.resize(3);
  randomState.state[0] = (dis(gen) * (max_x-min_x)) + min_x;  //set random value for each state varible
  randomState.state[1] = (dis(gen) * (max_y-min_y)) + min_y;
  randomState.state[2] = (dis(gen) * (max_yaw-min_yaw)) + min_yaw;
  return randomState;
}


RRTNode* tbiRRT::getNearestNode(NodeState q_rand, bool primary_tree) {
  RRTNode *q_near = NULL; 
  double dist = std::numeric_limits<double>::infinity(); 
  double new_dist;
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

RRTNode* tbiRRT::getNearestNode1(NodeState q_rand) {
  RRTNode *q_near = NULL; 
  double dist = std::numeric_limits<double>::infinity(); 
  double new_dist;
  for (auto n: tree1){ 
    new_dist = sqrt(pow(q_rand.state[0] - n->st.state[0],2) + pow(q_rand.state[1] - n->st.state[1],2)); 
    if (new_dist < dist) {
      q_near = n; 
      dist = new_dist;
      tree_num = true;  
    }
  } 
  for (auto n: tree2){ 
    new_dist = sqrt(pow(q_rand.state[0] - n->st.state[0],2) + pow(q_rand.state[1] - n->st.state[1],2)); 
    if (new_dist < dist) {
      q_near = n; 
      dist = new_dist;
      tree_num = false;
    }
  }   
  return q_near;
}

  void tbiRRT::expandNode1(const NodeState &q_rand, RRTNode *q_near, int relaxation_mode, bool direct){
  RRTNode q_new;
  double dist = std::numeric_limits<double>::infinity(); 
  double new_dist;
  bool is_new_node = false;
  bool ret_transition1 = false; 
  
  //from q_near apply random conmands   
  
  for (int i = 0; i < K; i++) {
    NodeState st = q_near->st;
    geometry_msgs::Twist command = m.generateRandomCommand();
    //     std::cout << "El comando es " << command <<std::endl;
      // ROS_INFO("En arbol 1 el q_near es X = %f, Y = %f , th = %f", q_near->st.state[0],q_near->st.state[1],q_near->st.state[2]); 
      cost_wheels_Qnew1 = m.integrateTransition(st, command, delta_t); // If relaxation_mode >= 1 --> allow two wheels

      // ROS_INFO("En arbol 1 el st, posible new nodo es X = %f, Y = %f , th = %f", st.state[0],st.state[1],st.state[2]);  

      if (cost_wheels_Qnew1 < 0.0) {
        continue;
        // 	std::cout << "Colision " <<std::endl;
      }

      ret_transition1 = transitionTest1 (q_near->st, st, q_rand);

      if(ret_transition1){
        // 	std::cout << "Se encuentra nodo sin colision " <<std::endl;
	      // get node with minimum distance
	      is_new_node = true;
	      new_dist = sqrt(pow(q_rand.state[0] - st.state[0],2) + pow(q_rand.state[1] - st.state[1],2));
        if (new_dist<dist) {
          q_new.st = st; 
          q_new.command_lin = command.linear.x;
          q_new.command_ang = command.angular.z;
          dist = new_dist;
          q_new.cost = cost_wheels_Qnew1; // New field:cost
        }
      }
      // ROS_INFO("En arbol 1 el st, new nodo es X = %f, Y = %f , th = %f", q_new.st.state[0],q_new.st.state[1],q_new.st.state[2]);  
  }
  //check if there is a new node and add it to the graph (unless its the goal)
  if (is_new_node){
    RRTNode *q_closest = areConnected(q_new.st, direct); 
    
    if(direct){
      q_new.parent = q_near;
      q_new.cost += q_near->cost;
      RRTNode *new_node = new RRTNode(q_new); 
      q_near->children.push_back(new_node);
      tree1.push_back(new_node);
      if(got_connected){
	      q_final_1 = new_node;
	      q_final_2 = q_closest;
        // q_final_1->cost = q_new.cost + q_near->cost;   // To add the costs of each node respect to father node
        // q_final_2->cost = q_closest->cost;
      }
    }     
    else{
      // 	std::cout << "hay un nodo indirecto " <<std::endl;
      q_near->command_lin = q_new.command_lin; 
      q_near->command_ang = q_new.command_ang;
      q_new.command_lin = 0;
      q_new.command_ang = 0;
      q_new.parent = q_near;
      q_new.cost += q_near->cost; // Accumulate the parent cost 
      RRTNode *new_node = new RRTNode(q_new); 
      q_near->children.push_back(new_node);
      tree2.push_back(new_node);
      if(got_connected){
	      q_final_2 = new_node;
	      q_final_1 = q_closest;
        // q_final_1->cost = q_new.cost + q_near->cost;   // To add the costs of each node respect to father node
        // q_final_2->cost = q_closest->cost;
      }      
    }
  }
}        

void tbiRRT::expandNode2(const NodeState &q_rand, RRTNode *q_near, int relaxation_mode, bool direct){
  RRTNode q_new;
  double dist = std::numeric_limits<double>::infinity(); 
  double new_dist;
  bool is_new_node = false;
  bool ret_transition2 = false; 

  
  for (int i = 0; i < K; i++) {
    NodeState st = q_near->st;
    geometry_msgs::Twist command = m.generateRandomCommand();
    //     std::cout << "El comando es " << command <<std::endl;

      //  ROS_INFO("En arbol 2 el q_near es X = %f, Y = %f , th = %f", q_near->st.state[0],q_near->st.state[1],q_near->st.state[2]); 
       cost_wheels_Qnew2 = m.integrateTransition2(st, command, -(delta_t)); // si direct es false, entonces le paso delta_t <0 a la integracion
      //  ROS_INFO("En arbol 2 el st, posible new nodo es X = %f, Y = %f , th = %f", st.state[0],st.state[1],st.state[2]);  

      if (cost_wheels_Qnew2 < 0.0) {
        continue;
          // 	std::cout << "Colision " <<std::endl;
      }
      ret_transition2 = transitionTest2 (q_near->st, st, q_rand);
      if(ret_transition2){
        // 	std::cout << "Se encuentra nodo sin colision " <<std::endl;
        // get node with minimum distance
        is_new_node = true;
        new_dist = sqrt(pow(q_rand.state[0] - st.state[0],2) + pow(q_rand.state[1] - st.state[1],2));
        if (new_dist<dist) {
          q_new.st = st; 
          q_near->command_lin = command.linear.x; //este dato se almacena en el hijo
          q_near->command_ang = command.angular.z;
          dist = new_dist;
          q_new.cost = cost_wheels_Qnew2; // New field:cost
        }
      }
      // ROS_INFO("En arbol 2 el st, new nodo es X = %f, Y = %f , th = %f", q_new.st.state[0],q_new.st.state[1],q_new.st.state[2]); 
  }
  if (is_new_node){
    RRTNode *q_closest = areConnected(q_new.st, direct); 
    
    if(direct){
      q_new.parent = q_near;
      q_new.cost += q_near->cost;
      RRTNode *new_node = new RRTNode(q_new); 
      q_near->children.push_back(new_node);
      tree1.push_back(new_node);
      if(got_connected){
	      q_final_1 = new_node;
	      q_final_2 = q_closest;
        // q_final_1->cost = q_new.cost + q_near->cost;   // To add the costs of each node respect to father node
        // q_final_2->cost = q_closest->cost;
      }
    }     
    else{
      q_near->command_lin = q_new.command_lin; 
      q_near->command_ang = q_new.command_ang;
      q_new.command_lin = 0;
      q_new.command_ang = 0;
      q_new.parent = q_near;
      q_new.cost += q_near->cost;
      RRTNode *new_node = new RRTNode(q_new); 
      q_near->children.push_back(new_node);
      tree2.push_back(new_node);
      if(got_connected){
	      q_final_2 = new_node;
	      q_final_1 = q_closest;
        // q_final_2->cost = q_new.cost + q_near->cost;   // To add the costs of each node respect to father node
        // q_final_1->cost = q_closest->cost;
      }      
    }
  }
}                            


bool tbiRRT::transitionTest1 (NodeState q_near, NodeState q_new, NodeState q_rand){ // Retorna un true o false diciendo si se acepta o no la configuracion

  // ROS_ERROR ("We ARE inside of the TRANSITION");
  double transition_probability;
  std::uniform_real_distribution<> dis_2(0,1);
  double random_prob = dis_2(gen);
  double cost_max = 1000000000;
  double cost_Qnear = calculateCostNear1(q_near, q_new, q_rand);
  double cost_Qnew = calculateCostNew1(q_near, q_new, q_rand, cost_wheels_Qnew1);
  double dist_Qnear_Qnew = sqrt(pow(q_near.state[0] - q_new.state[0],2) + pow(q_near.state[1] - q_new.state[1],2));;
  double slope_cost = (cost_Qnew - cost_Qnear)/dist_Qnear_Qnew;
  if (slope_cost > 0){
    transition_probability = exp (-(slope_cost)/(K_normal * Temp)); 
  }
  else {
    transition_probability = 1;
  } 
  if (cost_Qnew > cost_max){
    // ROS_ERROR("Can't be apply the transitionTest --> The cost_Qnew of new_node exceeds the maximum cost");
    return 0;
  }

  if(cost_Qnew < cost_Qnear ){ 
    // ROS_ERROR("cost_Qnew < cost_Qnear, por lo tanto transition probability = 1, se acepta inmediatamente la configuracion");   
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

bool tbiRRT::transitionTest2 (NodeState q_near, NodeState q_new, NodeState q_rand){ // Retorna un true o false diciendo si se acepta o no la configuracion

  // ROS_ERROR ("We ARE inside of the TRANSITION");
  double transition_probability;
  std::uniform_real_distribution<> dis_2(0,1);
  double random_prob = dis_2(gen);
  double cost_max = 1000000000;
  double cost_Qnear = calculateCostNear2(q_near, q_new, q_rand);
  double cost_Qnew = calculateCostNew2(q_near, q_new, q_rand, cost_wheels_Qnew2);
  double dist_Qnear_Qnew = sqrt(pow(q_near.state[0] - q_new.state[0],2) + pow(q_near.state[1] - q_new.state[1],2));;
  double slope_cost = (cost_Qnew - cost_Qnear)/dist_Qnear_Qnew;
  if (slope_cost > 0){
    transition_probability = exp (-(slope_cost)/(K_normal * Temp)); 
  }
  else {
    transition_probability = 1;
  }  
  if (cost_Qnew > cost_max){
    // ROS_ERROR("Can't be apply the transitionTest --> The cost_Qnew of new_node exceeds the maximum cost");
    return 0;
  }
  if(cost_Qnew < cost_Qnear ){ 
    // ROS_ERROR("cost_Qnew < cost_Qnear, por lo tanto transition probability = 1, se acepta inmediatamente la configuracion");   
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

double tbiRRT::calculateCostNear1(NodeState q_near, NodeState q_new, NodeState q_rand){

  double cost_dis_Qnear = sqrt(pow(q_rand.state[0] - q_near.state[0],2) + pow(q_rand.state[1] - q_near.state[1],2));
  double cost_wheels_Qnear = m.costWheelsQnear(q_near.state[0],q_near.state[1],q_near.state[2]);

  double ret_val = cost_dis_Qnear*1.5 + cost_wheels_Qnear;
  return ret_val;
}

double tbiRRT::calculateCostNew1(NodeState q_near, NodeState q_new, NodeState q_rand, double cost_wheels_Qnew){

  double cost_dist_Qnew = sqrt(pow(q_rand.state[0] - q_new.state[0],2) + pow(q_rand.state[1] - q_new.state[1],2));

  double ret_val = cost_dist_Qnew*1.5  + cost_wheels_Qnew;
  return ret_val;
}

double tbiRRT::calculateCostNear2(NodeState q_near, NodeState q_new, NodeState q_rand){

  double cost_dis_Qnear = sqrt(pow(q_rand.state[0] - q_near.state[0],2) + pow(q_rand.state[1] - q_near.state[1],2));
  double cost_wheels_Qnear = m.costWheelsQnear(q_near.state[0],q_near.state[1],q_near.state[2]);

  double ret_val = cost_dis_Qnear*1.5 + cost_wheels_Qnear;
  return ret_val;
}

double tbiRRT::calculateCostNew2(NodeState q_near, NodeState q_new, NodeState q_rand, double cost_wheels_Qnew){

  double cost_dist_Qnew = sqrt(pow(q_rand.state[0] - q_new.state[0],2) + pow(q_rand.state[1] - q_new.state[1],2));

  double ret_val = cost_dist_Qnew*1.5  + cost_wheels_Qnew;
  return ret_val;
}


RRTNode* tbiRRT::areConnected(NodeState st, bool direct) {
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
  // ROS_INFO("DADO QUE direct es: %d",direct);
  // ROS_INFO("El q_closest del arbol %d es X = %f, Y = %f , th = %f", !direct, q_closest->st.state[0],q_closest->st.state[1],q_closest->st.state[2]);  
  // ROS_INFO("El q_new del arbol %d es X = %f, Y = %f , th = %f",direct , st.state[0],st.state[1],st.state[2]);  
  got_connected = (fabs(st.state[0] - q_closest->st.state[0]) < goal_gap_m) && (fabs(st.state[1]-q_closest->st.state[1]) < goal_gap_m) && (fabs(st.state[2] - q_closest->st.state[2]) < goal_gap_rad);
  // ROS_INFO("The value of GOT_CONNECTED is: %d",got_connected);
  return q_closest;	 //devuelvo nodo mas cercano del otro arbol   
}


std::list<RRTNode> tbiRRT::getPath(){
  //get path from finished graph
  //   std::cout << "Entra en getpath " <<std::endl;
  std::cout << "Nodos en arbol de start: " << tree1.size() << std::endl;
  std::cout << "Nodos en arbol de goal: " << tree2.size() << std::endl;
  std::list<RRTNode> path;
  RRTNode* current_node = q_final_1;
  //   std::cout << "hasta aqui 1 " <<std::endl;
  path.push_front(*current_node); //incluyo q_final_1 pork sera el que tenga datos de los comandos. q_final_2 no tiene, solo sirve para enlazar con hijo 'real'
  //   std::cout << "hasta aqui 2 " <<std::endl;
  while (current_node->parent != NULL) {  
    current_node = current_node->parent;
    path.push_front(*current_node);
  }
  //   std::cout << "hasta aqui 3 " <<std::endl;
  current_node = q_final_2;
  path.push_front(*current_node);
  //   std::cout << "hasta aqui 4 " <<std::endl;
  while (current_node->parent != NULL) {  
    current_node = current_node->parent;
    path.push_back(*current_node);
  }
  //   std::cout << "sale de getpath " <<std::endl;
  return path;
}


visualization_msgs::MarkerArray tbiRRT::getPathMarker(const std::list< RRTNode >& path) 
{
  visualization_msgs::MarkerArray ret;
  visualization_msgs::Marker m_aux;

  m_aux.header.frame_id = this->m.getFrameID();
  m_aux.header.stamp = ros::Time::now();
  m_aux.action = visualization_msgs::Marker::DELETEALL;
  m_aux.points.clear();
  m_aux.type = visualization_msgs::Marker::POINTS;
  int cont = 0;
  NodeState pt;
  for (auto it = path.begin(); it != path.end(); it++, cont++) {
    if (cont > 0) { 
      pt = (--it)->st;
      it++;
      geometry_msgs::Twist command;
      command.linear.x = it->command_lin;
      command.angular.z = it->command_ang;

      if (cont % 5 == 0){  

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



visualization_msgs::Marker tbiRRT::getGraphMarker()
{
  visualization_msgs::Marker m;
  m.header.frame_id = this->m.getFrameID();
  m.header.stamp = ros::Time::now();
  m.ns = "tbirrt";
  m.action = visualization_msgs::Marker::ADD;
  m.pose.orientation.w = 1.0;
  m.id = 0;
  m.points.clear();
  m.type = visualization_msgs::Marker::POINTS;
  m.scale.x = 0.05;
  // Points are green
  visualization_msgs::Marker::_color_type color;
  color.g = 0;
  color.a = 1.0;
  geometry_msgs::Point p1;

  for (auto n : tree1){  
    color.r = 0;
    color.b = 1.0;
    p1.x = n->st.state[0];
    p1.y = n->st.state[1];
    
    m.points.push_back(p1); 
    m.colors.push_back(color);
  }
  for (auto n : tree2){  
    color.r = 1.0;
    color.b = 0;
    p1.x = n->st.state[0];
    p1.y = n->st.state[1];
    m.points.push_back(p1); 
    m.colors.push_back(color);   
  }
  m.lifetime = ros::Duration(2);
  return m;
}


double tbiRRT::retCostPath(const std::list< RRTNode >& path){
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
