#ifndef __COMMAND_EVALUATOR_CPP__
#define __COMMAND_EVALUATOR_CPP__

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/OccupancyGrid.h>
#include <boost/math/special_functions/sign.hpp>
#include <math.h>
#include <functions/RealVector.h>

#include "siar_footprint.hpp"


#ifndef MAP_IDX
#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))
#endif

namespace siar_controller {

  struct RobotCharacteristics {
    double a_max;
    double v_max;
    double theta_dot_max;
    double a_max_theta;
    double v_min;

    RobotCharacteristics () {
      a_max = 1.0;
      a_max_theta = 1.0;
      theta_dot_max = 1.0;
      v_max = 0.5;
      v_min = 0.05;
    }

    // Model parameters
    RobotCharacteristics (ros::NodeHandle &pn)
    {
      pn.param("a_max", a_max, 0.5);
      pn.param("a_max_theta", a_max_theta, 1.0);
      pn.param("v_max", v_max, 0.2);
      pn.param("omega_max", theta_dot_max, 0.4);
      pn.param("v_min", v_min, 0.05);
    }
  };

  class CommandEvaluator {
  public:
    CommandEvaluator(double w_dist, double w_safe, double T, const RobotCharacteristics &model, double delta_T = 0.1, SiarFootprint *footprint_p = NULL);

    CommandEvaluator(ros::NodeHandle &pn);

    inline void setWeightDistance(double w_d) {
      m_w_dist = w_d;

    }

    inline void setWidth(double width) {
        double cellsize = footprint_params->m_cellsize;
        double length = footprint_params->m_length;
        double w_width = footprint_params->m_wheel_width;
        bool simplified = footprint_params->m_simplified;
	double x_elec = footprint_params->m_x_elec;
	delete footprint_params;
        footprint_params = new SiarFootprint(cellsize, length, width, w_width, simplified, x_elec);
        delete footprint;
        footprint = NULL;
    }

    inline void setWidthElec(double width, double new_elec) {
        double cellsize = footprint_params->m_cellsize;
        double length = footprint_params->m_length;
        double w_width = footprint_params->m_wheel_width;
        bool simplified = footprint_params->m_simplified;
	delete footprint_params;
        footprint_params = new SiarFootprint(cellsize, length, width, w_width, simplified, new_elec);
        delete footprint;
        footprint = NULL;
    }

    inline void setWeightSafe(double w_s) {
      m_w_safe = w_s;
    }

    inline void restoreWeights() {
      m_w_dist = orig_m_w_dist;
      m_w_safe = orig_m_w_safe;
    }

    inline void setMinWheelLeft(double new_min_w_l) {
      min_wheel_left = new_min_w_l;
    }

    inline void setMinWheelRight(double new_min_w_r) {
      min_wheel_right = new_min_w_r;
    }

    inline double getMinWheelLeft() const {
      return min_wheel_left;
    }

    inline double getMinWheelRight() const {
      return min_wheel_right;
    }

    inline void setDeltaT(double new_delta_t){
        m_delta_T = new_delta_t;
    }

    //! @brief Sets the parameters into the evaluator
    //! @param footprint_p --> has the parameters for width, length and wheel_width
    //! @note The evaluator will delete the footprint_p
    void setParameters(double w_dist, double w_safe, double T, const RobotCharacteristics &model, double delta_T = 0.1, SiarFootprint *footprint_p = NULL);

    //! @brief Simulates the trajectory during T and generates a cost according to the cost map
    //! @retval -1.0 --> Collision  . If not collision --> cost of the trajectory (related to risk and difference with respect the user command)
    //! @param v_ini Initial velocity of the vehicle
    //! @param v_command  Effective Command which has to be tested
    //! @param operator_command User's commanded velocity (to evaluate the cost of the difference v_command and operator_command)
    //! @param alt_map Perceived altitude map of the environment
    //! @param m (out) Marker that can be used for representing the trajectory in RViz.
    double evaluateTrajectory(const geometry_msgs::Twist& v_ini, const geometry_msgs::Twist& v_command,
                              const geometry_msgs::Twist& operator_command, const nav_msgs::OccupancyGrid& alt_map,
                              visualization_msgs::Marker &m, double x = 0.0, double y = 0.0, double th = 0.0);

    //! @brief Same as EvaluateTrajectory, but when a collision is detected, it returns the minimum command without velocity that holds the same trajectory (same radius of curvature)
    double evaluateTrajectoryMinVelocity(const geometry_msgs::Twist& v_ini, geometry_msgs::Twist& v_command,
                                         const geometry_msgs::Twist& operator_command, const nav_msgs::OccupancyGrid& alt_map,
                                         visualization_msgs::Marker& m, double x = 0.0, double y = 0.0, double th = 0.0);

    //! @brief Simulates the trajectory during T and generates a cost according to the cost map
    //! @retval -1.0 --> Collision  . If not collision --> cost of the trajectory (related to risk and difference with respect the user command)
    //! @param v_ini Initial velocity of the vehicle
    //! @param v_command  Effective Command which has to be tested
    //! @param operator_command User's commanded velocity (to evaluate the cost of the difference v_command and operator_command)
    //! @param alt_map Perceived altitude map of the environment
    //! @param m (out) Marker that can be used for representing the trajectory in RViz.
    double evaluateTrajectoryRelaxed(const geometry_msgs::Twist& v_ini, const geometry_msgs::Twist& v_command,
                              const geometry_msgs::Twist& operator_command, const nav_msgs::OccupancyGrid& alt_map,
                              visualization_msgs::Marker &m, double x = 0.0, double y = 0.0, double th = 0.0);

    double evaluateTrajectoryTransition(const geometry_msgs::Twist& v_ini, const geometry_msgs::Twist& v_command,
                                                   const geometry_msgs::Twist& operator_command, const nav_msgs::OccupancyGrid& alt_map,
                                                   visualization_msgs::Marker& m, double x, double y, double th);

    inline double getCostWheelsQnew() const {return cost_wheels;}

    //! @brief Destructor
    ~CommandEvaluator();

    //! @brief Gets the characteristics of the model
    inline RobotCharacteristics getCharacteristics() const {return m_model;}

    inline SiarFootprint *getFootprint() {return footprint;}

    void initializeFootprint(const nav_msgs::OccupancyGrid& alt_map, visualization_msgs::Marker& m);
    void initializeFootprint(const nav_msgs::OccupancyGrid& alt_map);

    inline functions::RealVector getLastState() const {return last_state;}

     //! @brief Applies the footprint in the altitude map
    //! @param x X coord
    //! @param y Y coord
    //! @param th Theta angle
    //! @param alt_map The altitude map
    //! @param collision (Out parameter) Will become true if a collision is detected
    //! @param apply_collision If false, the wheel part of the footprint is used. If true, the collision part is used (and only positive obstacles are considered)
    int applyFootprint(double x, double y, double th,
                       const nav_msgs::OccupancyGrid &alt_map,
                       bool &collision, bool apply_collision = false);

    int applyFootprintRelaxed(double x, double y, double th, const nav_msgs::OccupancyGrid& alt_map, bool& collision);

    double applyFootprintTransition(double x, double y, double th, const nav_msgs::OccupancyGrid &alt_map, bool &collision, bool &collision_wheels);

    inline int point2index(double x, double y)
      {
       int ret_val;
       if (!rotate) {
         int i = (x - origin_x)*m_divRes;
         int j = (y - origin_y)*m_divRes;
         ret_val = MAP_IDX(width, i, j);
       } else {
         // From siar costmap
         ret_val =  ((int)((x-origin_x)*m_divRes))*width + width - (int)((y+origin_y)*m_divRes); //(int)(((x-origin_x)*m_divRes + 1)*width) - (int)((y - origin_y)*m_divRes);
       }
      if (ret_val >= total_points) {
        ret_val = -1;
      }
        return ret_val;
      }

  protected:
    double m_T; // Lookahead time
    double m_delta_T; // Timestep
    double m_divRes, origin_x, origin_y;
    int width; bool rotate; int total_points;
    int positive_obs, negative_obs;
    double min_wheel_left, min_wheel_right;
    double cost_wheels;

    double m_w_dist, m_w_safe; // Different weights. Respectively: Distance to commanded velocity, safety, collision penalty
    double orig_m_w_dist, orig_m_w_safe; // Different weights. Respectively: Distance to commanded velocity, safety, collision penalty

    bool consider_two_wheels; // Flags: two_wheels: if true considers two wheels when turning in an intersection
					     // Body: checks the body of the platform for positive obstacles

    RobotCharacteristics m_model;

    SiarFootprint *footprint, *footprint_params;

    functions::RealVector last_state;


    //! @brief Actualizes the value of linear and angular velocities according to the model of the vehicle
    //! @param v Linear velocity (in-out) (m/s)
    //! @param w Angular velocity (in-out) (rad/s)
    //! @param dt Step time (s)
    //! @param com Commanded velocity (m/s and rad/s)
    inline void computeNewVelocity(double &v, double &w, double dt, const geometry_msgs::Twist &com){
        v = std::min(com.linear.x, v + m_model.a_max * dt * boost::math::sign(com.linear.x - v));
        w = std::min(com.angular.z, w + m_model.a_max_theta * dt * boost::math::sign(com.angular.z - w));
      }

    inline void setParams(const nav_msgs::OccupancyGrid &alt_map) {
      rotate = alt_map.info.origin.orientation.w < 0.9999;
      origin_x = alt_map.info.origin.position.x;
      origin_y = alt_map.info.origin.position.y;
      m_divRes = 1.0 / alt_map.info.resolution;
      width = alt_map.info.width;
      total_points = alt_map.info.height * width;
//       if (rotate)
//         ROS_INFO("Set params: Rotated map");
//       else
//         ROS_INFO("Set params: Straight map");
    }


    // inline int point2index(double x, double y)
    //     {
    //       int ret_val;

    //       if (!rotate) {
    //         int i = (x - origin_x)*m_divRes;
    //         int j = (y - origin_y)*m_divRes;
    //         ret_val = MAP_IDX(width, i, j);
    //       } else {
    //         // From siar costmap
    //         ret_val =  ((int)((x-origin_x)*m_divRes))*width + width - (int)((y+origin_y)*m_divRes); //(int)(((x-origin_x)*m_divRes + 1)*width) - (int)((y - origin_y)*m_divRes);
    //       }

    //       if (ret_val >= total_points) {
    //         ret_val = -1;
    //       }

    //       return ret_val;
    //     }
  };

CommandEvaluator::CommandEvaluator(double w_dist, double w_safe, double T, const RobotCharacteristics &model, double delta_T, SiarFootprint *footprint_p):footprint(NULL), footprint_params(NULL)
{

  setParameters(w_dist, w_safe, T, model, delta_T, footprint_p);
  orig_m_w_dist = w_dist;
  orig_m_w_safe = w_safe;
  positive_obs = 127;
  negative_obs = -127;
  ROS_INFO("Created the evaluater. Positive obstacle: %d", positive_obs);
}

CommandEvaluator::CommandEvaluator(ros::NodeHandle& pn):m_model(pn),footprint(NULL)
{

  pn.param("w_dist", m_w_dist, 0.0);
  pn.param("w_safe", m_w_safe, 1.0);
  pn.param("delta_T", m_delta_T, 0.2);
  pn.param("T", m_T, 3.0);
  pn.param("positive_obs", positive_obs, 127);
  pn.param("negative_obs", negative_obs, -127);
  pn.param("min_wheel_l", min_wheel_left, 0.2); // Minimum fragment of the wheel that has to be without obstacle to be collision-free (in relaxed mode)
  pn.param("min_wheel_r", min_wheel_right, 0.2); // Minimum fragment of the wheel that has to be without obstacle to be collision-free (in relaxed mode)
  pn.param("consider_two_wheels", consider_two_wheels, true); // If true --> considers the cost of two wheels when turning in a intersection (if not only considers the one that remains in the floor)
  footprint_params = new SiarFootprint(pn);
  orig_m_w_dist = m_w_dist;
  orig_m_w_safe = m_w_safe;

  ROS_INFO("Created the evaluator. Positive obstacle: %d", positive_obs);
}



void CommandEvaluator::setParameters(double w_dist, double w_safe, double T, const RobotCharacteristics &model, double delta_T, SiarFootprint *footprint_p) {
  m_w_dist = w_dist;
  m_w_safe = w_safe;
  m_T = T;
  m_delta_T = delta_T;
  m_model = model;
  delete footprint_params;
  footprint_params = footprint_p;
  ROS_INFO("Command Evaluator::Set parameters --> w_dist = %f\t w_safe=%f\t T=%f\t D_T=%f", w_dist, w_safe, T, delta_T);
}

CommandEvaluator::~CommandEvaluator()
{
  delete footprint;
  delete footprint_params;
}

double CommandEvaluator::evaluateTrajectory(const geometry_msgs::Twist& v_ini, const geometry_msgs::Twist& v_command,
                                            const geometry_msgs::Twist& operator_command, const nav_msgs::OccupancyGrid& alt_map,
                                            visualization_msgs::Marker &m, double x, double y, double th)
{
  double dt = m_delta_T;
  int steps = std::abs(m_T / m_delta_T);  //valor absoluto para evaluar tambien delta negativo

  setParams(alt_map);

  double lv = v_command.linear.x; //de momento uso esto, ya que la v_ini que se tomaba no tenia mucha logica en el calculo de computeNewVelocity
  double av = v_command.angular.z;

  // ROS_INFO ("Evaluating traj. lv = %f, av =%f",lv,av);

  int cont_footprint = 0;

  // Initialize the footprint if needed:
  initializeFootprint(alt_map, m);

  bool collision = false;
  for(int i = 0; i < steps && !collision; i++)
  {
    // Integrate the model
    double lin_dist = lv * dt;
    if (dt > 0){
      th = th + (av * dt);
      x = x + lin_dist*cos(th); // Euler 1
      y = y + lin_dist*sin(th);
    }
    else{
      x = x + lin_dist*cos(th); // Euler 1
      y = y + lin_dist*sin(th);
      th = th + (av * dt); //th is update after
    }

    // Represent downsampled
    if (i % 10 == 0)
      footprint->addPoints(x, y, th, m, 0, i == 0);

    // Update cost
    cont_footprint += applyFootprint(x, y, th, alt_map, collision);

    if (!collision )
      applyFootprint(x, y, th, alt_map, collision, true); // Search for positive collisions too
  }

  double ret = cont_footprint * m_w_safe + m_w_dist * sqrt(pow(x - operator_command.linear.x * m_T, 2.0) + y*y);  // m_w_safe =1 (Different weights. Respectively: Distance to commanded velocity, safety, collision penalty
  //m_T =3 , lookhead time , x e y son los valores de q_near que son evaluados en esta funcion para evaluar trayectoria
  if (collision) {
    m.color.r = 1.0;
    m.color.g = 0.0;
    return -1.0;
  }

  last_state.resize(3);
  last_state[0] = x;
  last_state[1] = y;
  last_state[2] = th;

  return ret;
}

double CommandEvaluator::evaluateTrajectoryMinVelocity(const geometry_msgs::Twist& v_ini, geometry_msgs::Twist& v_command, const geometry_msgs::Twist& operator_command,
                                                       const nav_msgs::OccupancyGrid& alt_map, visualization_msgs::Marker& m, double x, double y, double th)
{
  double dt = m_delta_T;
//   int steps = m_T / m_delta_T;
  int steps = std::abs(m_T / m_delta_T);  //valor absoluto para evaluar tambien delta negativo

  double lv = v_ini.linear.x;
  double av = v_ini.angular.z;
  setParams(alt_map);

  int cont_footprint = 0;

  // Initialize the footprint if needed and clear marker:
  initializeFootprint(alt_map, m);

  bool collision = false;
  double acc_dist = 0.0;
  double t = 0.0;
  for(int i = 0; i <= steps && !collision; i++)
  {
    computeNewVelocity(lv, av, dt, v_command);

    // Integrate the model
    double lin_dist = lv * dt;
    acc_dist += lin_dist;
    t += m_T;

    if (dt > 0){
      th = th + (av * dt);
      x = x + lin_dist*cos(th); // Euler 1
      y = y + lin_dist*sin(th);
    }
    else{
      x = x + lin_dist*cos(th); // Euler 1
      y = y + lin_dist*sin(th);
      th = th + (av * dt); //actualizo th despues
    }

    // Represent downsampled
    if (i % 5 == 0)
      footprint->addPoints(x, y, th, m, 0, i == 0);

    // Actualize the cost
    cont_footprint += applyFootprint(x, y, th, alt_map, collision);
    if (!collision)
      applyFootprint(x, y, th, alt_map, collision, true); // Search for positive collisions too
  }

  double ret = cont_footprint * m_w_safe + m_w_dist * sqrt(pow(x - operator_command.linear.x * m_T, 2.0) + y*y);
  if (collision) {
    double v_x = acc_dist / (t - 2.0 * m_T);

    if (fabs(v_x) < m_model.v_min) { // Check the minimum velocity
      m.color.r = 1.0;
      m.color.g = 0.0;
      return -1.0;
    } else {
      ROS_INFO("v_x = %f", v_x);
      v_command.angular.z *= v_x / v_command.linear.x; // Maintain the commanded radius
      v_command.linear.x = v_x;
    }
  }

  last_state.resize(3);
  last_state[0] = x;
  last_state[1] = y;
  last_state[2] = th;

  return ret;
}

double CommandEvaluator::evaluateTrajectoryRelaxed(const geometry_msgs::Twist& v_ini, const geometry_msgs::Twist& v_command,
                                                   const geometry_msgs::Twist& operator_command, const nav_msgs::OccupancyGrid& alt_map,
                                                   visualization_msgs::Marker& m, double x, double y, double th)
{
  double dt = m_delta_T;
  int steps = std::abs(m_T / m_delta_T);  

  double lv = v_command.linear.x; 
  double av = v_command.angular.z;

  setParams(alt_map);

  int cont_footprint = 0;

  m.points.clear();
  // Initialize the footprint if needed and clear marker:
  initializeFootprint(alt_map, m);

  bool collision = false;
  bool collision_relax = false;
  for(int i = 0; i <= steps && !collision; i++)
  {

    // Integrate the model
    double lin_dist = lv * dt;

    if (dt > 0){
      th = th + (av * dt);
      x = x + lin_dist*cos(th); // Euler 1
      y = y + lin_dist*sin(th);
    }
    else{
      x = x + lin_dist*cos(th); // Euler 1
      y = y + lin_dist*sin(th);
      th = th + (av * dt); 
    }

    // Represent downsampled
    if (i % 5 == 0)
      footprint->addPoints(x, y, th, m, 0, i == 0);

    // Actualize the cost
    cont_footprint += applyFootprintTransition(x, y, th, alt_map, collision,collision_relax);
    collision = collision | collision_relax;

    if (!collision )
      applyFootprint(x, y, th, alt_map, collision, true); // Search for positive collisions too
  }

  double ret = cont_footprint * m_w_safe + m_w_dist * sqrt(pow(x - operator_command.linear.x * m_T, 2.0) + y*y);
  if (collision) {
    m.color.r = 1.0;
    m.color.g = 0.0;
    return -1.0;
  }

  last_state.resize(3);
  last_state[0] = x;
  last_state[1] = y;
  last_state[2] = th;

  return ret;
}

double CommandEvaluator::evaluateTrajectoryTransition(const geometry_msgs::Twist& v_ini, const geometry_msgs::Twist& v_command,
                                                   const geometry_msgs::Twist& operator_command, const nav_msgs::OccupancyGrid& alt_map,
                                                   visualization_msgs::Marker& m, double x, double y, double th)
{
  double dt = m_delta_T;
  int steps = std::abs(m_T / m_delta_T); //m_T =3 y m_delta_T= 0.2 => step =15

  double lv = v_command.linear.x;
  double av = v_command.angular.z;

  setParams(alt_map);

  double cont_footprint = 0;
  double ret = 0;

  // Initialize the footprint if needed and clear marker:
  initializeFootprint(alt_map, m);

  bool collision = false;
  bool collision_wheels =false;
  for(int i = 0; i <= steps && !collision; i++){ //Here is the beginin of the model integration
    double lin_dist = lv * dt;

    if (dt > 0){
      th = th + (av * dt);
      x = x + lin_dist*cos(th); // Euler 1
      y = y + lin_dist*sin(th);
    }
    else{
      x = x + lin_dist*cos(th); // Euler 1
      y = y + lin_dist*sin(th);
      th = th + (av * dt); //th is update after
    }
    // Represent downsampled
    if (i % 5 == 0)
      footprint->addPoints(x, y, th, m, 0, i == 0);

    cont_footprint += applyFootprintTransition(x, y, th, alt_map, collision, collision_wheels);
    if (!collision )
      applyFootprint(x, y, th, alt_map, collision, true); // Search for positive collisions too
  }

  last_state.resize(3);
  last_state[0] = x;
  last_state[1] = y;
  last_state[2] = th;

  if (collision) {
    m.color.r = 1.0;
    m.color.g = 0.0;
    return -1;
  }

  ret = cont_footprint;

  return ret;
}


int CommandEvaluator::applyFootprint(double x, double y, double th,
                                     const nav_msgs::OccupancyGrid &alt_map,
                                     bool &collision, bool apply_collision)
{
  int ret_val = 0;

  FootprintType fp;
  if (!apply_collision)
    fp = footprint->getFootprint(x, y, th);
  else
    fp = footprint->getFootprintCollision(x, y, th);

  collision = false;

  int index;
  for (unsigned int i = 0; i < fp.size() && !collision; i++) {

    index = point2index(fp[i].x, fp[i].y);
    // ROS_INFO("Point2index(%f,%f) = %d,%d. Value = %d", fp.at(i).x, fp.at(i).y,index/alt_map.info.width, index%alt_map.info.width, alt_map.data[index]);

    if (index < 0) {
      continue;
    }
    if (alt_map.data[index] == positive_obs || (alt_map.data[index] == negative_obs && !apply_collision))
      collision = true; // Collision detected! (if applying the collision map, only consider positive obstacles)

    ret_val += abs(alt_map.data[index]);

  }

  return ret_val;
}

int CommandEvaluator::applyFootprintRelaxed(double x, double y, double th,
                                     const nav_msgs::OccupancyGrid &alt_map,
                                     bool &collision)
{
  int ret_val = 0;

  FootprintType fp = footprint->getFootprint(x, y, th);
  FootprintType orig = footprint->getFootprint(0, 0 , 0);

  collision = false;

  int index;

  bool right_wheel = false;
  bool left_wheel = false;
  int size = fp.size();
  int cont_left = (size) / 2;
  int cont_right = (size) / 2;


  for (unsigned int i = 0; i < size && !collision; i++) {

    index = point2index(fp[i].x, fp[i].y);
//     ROS_INFO("Point2index(%f,%f) = %d,%d. Value = %d", fp.at(i).x, fp.at(i).y,index/alt_map.info.width, index%alt_map.info.width, alt_map.data[index]);

    if (index < 0) {
      continue;
    }
    if (alt_map.data[index] == positive_obs ) {
      collision = true;
      ret_val = -1;
    }
    else if (alt_map.data[index] == negative_obs || alt_map.data[index] == positive_obs) {
      if (orig[i].y > 0.0) {
        left_wheel = true;
        cont_left--;

        if (min_wheel_left + 0.01 > min_wheel_right || consider_two_wheels) {  //min_wheel_left = min_wheel_right = 0.2
          ret_val += abs(alt_map.data[index]);
        }
      }
      else {
        right_wheel = true;
        cont_right--;

        if (min_wheel_right + 0.01 > min_wheel_left || consider_two_wheels) {
          ret_val += abs(alt_map.data[index]);
        }
      }
    }
  }

  collision |= ( (double)cont_left / (double)size * 2.0 ) < min_wheel_left;
  collision |= ( (double)cont_right / (double)size * 2.0 ) < min_wheel_right;

  return ret_val;
}

double CommandEvaluator::applyFootprintTransition(double x, double y, double th,
                                              const nav_msgs::OccupancyGrid &alt_map,
                                              bool &collision, bool &collision_relaxed)
{

  int ret_val = 0;

  FootprintType fp = footprint->getFootprint(x, y, th);
  FootprintType orig = footprint->getFootprint(0, 0 , 0);
  collision = false;
  collision_relaxed = false;

  int index;


  int size = fp.size();
  double porctj_left_1 = 0, porctj_left_2 = 0, porctj_left_3 = 0, porctj_right_1 = 0, porctj_right_2 = 0, porctj_right_3 = 0,cont_porctj_left = 0, cont_porctj_right = 0;
  double cont_Coll_pix_left_1 = 0, cont_Coll_pix_left_2 = 0 ,cont_Coll_pix_left_3 = 0, cont_Coll_pix_right_1 = 0, cont_Coll_pix_right_2 = 0, cont_Coll_pix_right_3 = 0;
  double cont_Tot_pix_left_1 = 0, cont_Tot_pix_left_2 = 0 ,cont_Tot_pix_left_3 = 0, cont_Tot_pix_right_1 = 0, cont_Tot_pix_right_2 = 0, cont_Tot_pix_right_3 = 0;

  double wheel_length = footprint->m_length / 3.0;
  double wl_2 = wheel_length * 0.5;

  for (unsigned int i = 0; i < size && !collision; i++) {
    index = point2index(fp[i].x, fp[i].y);
    if (index < 0)
      continue;

    ret_val += abs(alt_map.data[index]);

    if (orig[i].y > 0.0) {
      if (orig[i].x > wl_2){
        cont_Tot_pix_left_1 ++;
      } else if (orig[i].x < -wl_2){
        cont_Tot_pix_left_3  ++;
      } else {
        cont_Tot_pix_left_2  ++;
      }
    }
    else {
      if (orig[i].x > wl_2){
        cont_Tot_pix_right_1 ++;
      } else if (orig[i].x < -wl_2){
       cont_Tot_pix_right_3 ++;
      } else {
        cont_Tot_pix_right_2 ++;
      }
    }

    if (alt_map.data[index] == positive_obs ) {
      collision = true;
      ret_val = -1;
    }
    else if (alt_map.data[index] == negative_obs)
    {
      if (orig[i].y > 0.0) {
        if (orig[i].x > wl_2){
          cont_Coll_pix_left_1 ++;
        }
        else if (orig[i].x < -wl_2){
          cont_Coll_pix_left_3 ++;
        } else {
          cont_Coll_pix_left_2 ++;
        }
      }
      else {
        if (orig[i].x > wl_2){
          cont_Coll_pix_right_1 ++;
        } else if (orig[i].x < -wl_2) {
          cont_Coll_pix_right_3 ++;
        } else {
          cont_Coll_pix_right_2 ++;
        }
      }
    }
  }
  if ((cont_Coll_pix_left_1 / cont_Tot_pix_left_1)> 0.8){
    porctj_left_1 = 1;
  }
  if ((cont_Coll_pix_left_2 / cont_Tot_pix_left_2)> 0.8){
    porctj_left_2 = 1;
  }
  if ((cont_Coll_pix_left_3 / cont_Tot_pix_left_3)> 0.8){
    porctj_left_3 = 1;
  }
  if ((cont_Coll_pix_right_1 / cont_Tot_pix_right_1)> 0.8){
    porctj_right_1 = 1;
  }
  if ((cont_Coll_pix_right_2 / cont_Tot_pix_right_2)> 0.8){
    porctj_right_2 = 1;
  }
  if ((cont_Coll_pix_right_3 / cont_Tot_pix_right_3)> 0.8){
    porctj_right_3 = 1;
  }

  cont_porctj_left = porctj_left_1 + porctj_left_2 + porctj_left_3;
  cont_porctj_right = porctj_right_1 + porctj_right_2 + porctj_right_3;

  

  if ((cont_porctj_left >2 && cont_porctj_right > 2)||(cont_porctj_left >=3 || cont_porctj_right >= 3)){
    // ROS_INFO("Cont pcj left 1: %f\t2: %f\t3: %f\tnegative obs:%d", cont_Coll_pix_left_1, cont_Coll_pix_left_2, cont_Coll_pix_left_3, negative_obs);
    // ROS_INFO("Cont pcj right 1: %f\t2: %f\t3: %f\tnegative obs:%d", cont_Coll_pix_right_1, cont_Coll_pix_right_2, cont_Coll_pix_right_3, negative_obs);
    collision = true;

    return -1;
  }

  int cont_left = cont_Coll_pix_left_1 + cont_Coll_pix_left_2 + cont_Coll_pix_left_3;
  int cont_right = cont_Coll_pix_right_1 + cont_Coll_pix_right_2 + cont_Coll_pix_right_3;
  size*=2;
  collision_relaxed = ( 1 - (double)cont_left / (double)size ) < min_wheel_left;
  collision_relaxed |= ( 1 - (double)cont_right / (double)size ) < min_wheel_right;

  return ret_val;
}



void CommandEvaluator::initializeFootprint(const nav_msgs::OccupancyGrid& alt_map, visualization_msgs::Marker& m)
{
  m.points.clear();
  initializeFootprint(alt_map);

}

void CommandEvaluator::initializeFootprint(const nav_msgs::OccupancyGrid& alt_map)
{
  if (footprint == NULL) {
      ROS_INFO("Getting footprint. Resolution: %f", alt_map.info.resolution);
    if (footprint_params == NULL)
       // TODO: CONFIGURABLE FOR VARIABLE WIDTH PROTOTYPE
      footprint = new SiarFootprint(alt_map.info.resolution);
    else
      footprint = new SiarFootprint(alt_map.info.resolution, footprint_params->m_length, footprint_params->m_width, footprint_params->m_wheel_width, true);

    ROS_INFO("Alt map: height = %d \t width = %d", alt_map.info.height, alt_map.info.width);
  }
}


}


#endif // __COMMAND_EVALUATOR_CPP__