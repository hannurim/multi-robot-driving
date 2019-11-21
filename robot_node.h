#ifndef ROBOT_NODE_H
#define ROBOT_NODE_H

#include <iostream>
#include <ros/ros.h>
#include <vector>
#include <math.h>
#include <algorithm>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>

#define PI      3.141592
#define R_ANG   1.570796    // Reference angle[rad]
#define R_DIS   0.2         // Reference distance[m]
#define Kw      0.8         // Gain radian velocity
#define Kv      0.3         // Gain distance velocity
#define Kl      0.8         // Weights for updating Robot position

using namespace std;

class Robot {
public:
    bool    valid;
    pair<double, double> goal;
    Robot() { valid = false; goal = make_pair(0,0); }
};

int GetMin(vector<pair<double, double> > list)
{
    double  min     = 99999;
    int     index   = -1;
    for (vector<int>::size_type i = 0; i < list.size(); i++)
    // store the data of the nearest Robot2
    {
        double dist = sqrt(pow(list[i].first,2) + pow(list[i].second,2));
        if (min > dist) {
            min = dist;
            index = i;
        }
    }
    return index;
}

double Tx12 = -0.3, Ty12 = -0.1;    // Target that Robot2 should follow from the lead robot of view
double Tx13 = 0.3, Ty13 = -0.1;     // Target that Robot3 should follow from the lead robot of view
double Theta12, Theta21;            // The angle of Robot2 as seen by the lead robot/The angle of lead robot as seen by Robot2
double Theta13, Theta31;            // The angle of Robot3 as seen by the lead robot/The angle of lead robot as seen by Robot3

#endif // ROBOT_NODE_H
