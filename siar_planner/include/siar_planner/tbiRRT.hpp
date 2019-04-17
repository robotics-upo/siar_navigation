#ifndef _tbiRRT_HPP_
#define _tbiRRT_HPP_

#include "siar_planner/Planner.hpp"

#include "siar_planner/NodeState.hpp"
#include "siar_planner/RRTNode.h"
#include "siar_planner/SiarModel.hpp"

#include "visualization_msgs/Marker.h"
#include <visualization_msgs/MarkerArray.h>
#include <functions/functions.h>
#include <math.h>

#include <ros/ros.h>

class tbiRRT:public Planner
{
public:
  tbiRRT(ros::NodeHandle &nh, ros::NodeHandle &pnh);
  
  ~tbiRRT();
  
  virtual double resolve(NodeState start, NodeState goal, std::list<RRTNode>& path); 
  
  virtual visualization_msgs::Marker getGraphMarker();
  
  double getDeltaT() const {return delta_t;}
  
  std::list<RRTNode *> tree1;
  std::list<RRTNode *> tree2;
  RRTNode *q_final_1 = NULL;
  RRTNode *q_final_2 = NULL;

  virtual int getGraphSize() {
    return tree1.size() + tree2.size();
  }
  
protected:
  virtual void clear();
  
  RRTNode* areConnected(NodeState st, bool direct);
  bool got_connected = false;
  
  RRTNode goal_node;
  RRTNode start_node;
   
  //new functions
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
  double K_normal, Temp_init, Temp_1,Temp_2; //(cost_initial - cost_goal)/2
  int alfa , nFail, nFailmax ;

  double cost_wheels_Qnew1, cost_wheels_Qnew2;
  double cost_Qnear, cost_Qnew, cost_wheels_Qnew;
};

tbiRRT::tbiRRT(ros::NodeHandle &nh, ros::NodeHandle &pnh):Planner(nh, pnh)
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
}

tbiRRT::~tbiRRT(){
  clear();
  
}


inline void tbiRRT::clear()
{
  for (auto n:tree1) {
    delete n;
  }
  for (auto n:tree2) {
    delete n;
  }
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
  Temp_1=Temp_init;
  Temp_2=Temp_init;
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
    }
  } 
  for (auto n: tree2){ 
    new_dist = sqrt(pow(q_rand.state[0] - n->st.state[0],2) + pow(q_rand.state[1] - n->st.state[1],2)); 
    if (new_dist < dist) {
      q_near = n; 
      dist = new_dist;
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
      // q_new.cost += q_near->cost;
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
      // q_new.cost += q_near->cost; // Accumulate the parent cost 
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
      // q_new.cost += q_near->cost;
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
      // q_new.cost += q_near->cost;
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
    transition_probability = exp (-(slope_cost)/(K_normal * Temp_1)); 
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
    Temp_1 = Temp_1/ alfa;
    nFail = 0;
    return 1;
  }
  else{
    if (nFail > nFailmax){  // Set in 0 and 20 respectively
      Temp_1 = Temp_1 * 10 *alfa;
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
    transition_probability = exp (-(slope_cost)/(K_normal * Temp_2)); 
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
    Temp_2 = Temp_2/ alfa;
    nFail = 0;
    return 1;
  }
  else{
    if (nFail > nFailmax){  // Set in 0 and 20 respectively
      Temp_2 = Temp_2 * 10 *alfa;
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

#endif
