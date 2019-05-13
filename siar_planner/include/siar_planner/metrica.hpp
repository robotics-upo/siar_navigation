#ifndef _METRICA_HPP_
#define _METRICA_HPP_

#include "siar_planner/RRTNode.h"

#include <functions/functions.h>
#include <math.h>

#include <ros/ros.h>

class Metrica
{
public:
    Metrica(ros::NodeHandle &nh, ros::NodeHandle &pnh);

    inline double metrica3D (const RRTNode &q_node_1, const RRTNode &q_node_2);
    double metrica3D (const NodeState &s1, const NodeState &s2);
    double getAngDist(double a, double b) const;

protected:

    double distance, dif_angle, metrica;
    double K_metrica;

};

Metrica::Metrica(ros::NodeHandle &nh, ros::NodeHandle &pnh){
    if (!pnh.getParam("K_metrica", K_metrica)) {
       K_metrica = 0;
    }
    ROS_INFO("In Metrica the value of K = %f", K_metrica);
}


double Metrica::metrica3D (const RRTNode &q_node_1, const RRTNode &q_node_2){
    return metrica3D(q_node_1.st, q_node_2.st);
}

double Metrica::metrica3D (const NodeState &s1, const NodeState &s2){
    distance = sqrt(pow(s1.state[0] - s2.state[0],2) + pow(s1.state[1] - s2.state[1],2));
    dif_angle = getAngDist(s1.state[2], s2.state[2]);
    metrica = distance + K_metrica * dif_angle;

    return metrica;

}

double Metrica::getAngDist(double a, double b) const {
    double dif_angle = fabs(a - b);
    if (dif_angle > 2*M_PI) {
        dif_angle = dif_angle - floor(0.5*dif_angle/M_PI)*2*M_PI; // Reduces the difference into [0,2*PI]
    }
    if (dif_angle > M_PI) {
        dif_angle = 2*M_PI - dif_angle; // Then to 0 to PI
    }

    return dif_angle;
}

#endif

 