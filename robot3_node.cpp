#include "robot_node.h"

class Robot     r1;     // Master(Robot1) data

// Goal<x,y>
vector<pair<double, double> > r_list;       // List of possible Robots position
vector<pair<double, double> > r1_list;      // List of possible Robot1 position

// Goal<x,y>
vector<pair<double, double> > clustered;    // Clustered data

// Scan data converted to <x,y>
vector<pair<double,double> >  raw_data;
pair<double,double>           s;
pair<double,double>           f;
pair<double,double>           goal;

double  dist;
bool    start_flag = true;

geometry_msgs::Twist velOutput;

void Association();     // Function to track the position of the Master Robot
void Identification();  // Index the Master Robot only once
void GettingRobot();    // Function responsible for driving

void sensorCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    int     total_deg   = msg->angle_max / msg->angle_increment;
    for (int i = 0; i < (total_deg+1); i++)
    // Coordination
    {
        int deg = (i + 180) % (total_deg + 1);
        if (msg->ranges[deg] && msg->ranges[deg] < 1.0) { // Leave 0~1.0m data
            s   = make_pair(-1 * msg->ranges[deg] * sin(deg*PI/180), msg->ranges[deg] * cos(deg*PI/180));
            raw_data.push_back(s);
        }
    }
    for (vector<int>::size_type i = 0; i < raw_data.size(); i++)
    // Clustering
    {
        dist = sqrt(pow(raw_data[i].first-raw_data[i+1].first, 2) +
                    pow(raw_data[i].second-raw_data[i+1].second, 2));
        if (start_flag && dist < 0.1) {
            s           = make_pair(raw_data[i].first,raw_data[i].second);
            start_flag  = false;
        }
        else if (!start_flag && dist > 0.1) {
            f = make_pair(raw_data[i].first,raw_data[i].second);
            double width = sqrt(pow(s.first-f.first,2) + pow(s.second-f.second,2));
            if (width < 0.2 && width > 0.05) { // Save data that get 0.05~0.2m width
                goal = make_pair((s.first + f.first) / 2, (s.second + f.second) / 2);
                clustered.push_back(goal);
            }
            start_flag   = true;
        }
    }
    if (r1.goal.first && clustered.size() > 0) Association();
    else if (!r1.goal.first && clustered.size() >= 2) Identification();
    GettingRobot();
    printf("goal = (%lf, %lf)\n", r1.goal.first, r1.goal.second);
    if (!raw_data.empty()) raw_data.clear();
    if (!clustered.empty()) clustered.clear();
    if (!r_list.empty()) r_list.clear();
    if (!r1_list.empty()) r1_list.clear();
}

void robot1Callback(const geometry_msgs::PointConstPtr& msg)
// Relative coordinates of Robot3 published by Master Robot
{
    Theta13 = atan2(msg->y, msg->x);
}

void Association()
// Update to the nearest value within 0.3m of existing value
{
    for (vector<int>::size_type i = 0; i < clustered.size(); i++) {
        double r1_dist = sqrt(pow(clustered[i].first-r1.goal.first, 2) +
                              pow(clustered[i].second-r1.goal.second, 2));
        if (r1_dist < 0.3)
            r1_list.push_back(clustered[i]);
    }
    if (!r1_list.empty()) {
        int index = GetMin(r1_list);
        r1.goal.first = r1.goal.first * Kl + r1_list[index].first * (1 - Kl);   // Smoothing filter
        r1.goal.second = r1.goal.second * Kl + r1_list[index].second * (1 - Kl);
    }
}

void Identification()
// Index which is Master Robot and Robot2
{
    int index = GetMin(clustered);
    r_list.push_back(clustered[index]);
    clustered.erase(clustered.begin()+index);
    index = GetMin(clustered);
    r_list.push_back(clustered[index]);
    double  r1_theta = atan2(r_list[0].second, r_list[0].first);
    double  r3_theta = atan2(r_list[1].second, r_list[1].first);
    int     r1_index;
    if (abs(r1_theta - r3_theta) < PI) {
        if (r1_theta < r3_theta) {
            r1_index = 0;
        }
        else {
            r1_index = 1;
        }
    }
    else {
        if (r1_theta > r3_theta) {
            r1_index = 0;
        }
        else {
            r1_index = 1;
        }
    }
    r1.goal = r_list[r1_index];
}

void GettingRobot()
// Driving
{
    Theta31 = atan2(r1.goal.second, r1.goal.first);
    double diff = Theta31 - (Theta13 + PI);
    goal.first = r1.goal.first + cos(diff) * Tx13 - sin(diff) * Ty13;
    goal.second = r1.goal.second + sin(diff) * Tx13 + cos(diff) * Ty13;
    double theta_target = atan2(goal.second, goal.first);
    double theta_error = theta_target - R_ANG;
    double dist_target = goal.second;
    double dist_error = dist_target - R_DIS;
    velOutput.angular.z = Kw * theta_error;
    velOutput.linear.x = Kv * dist_error;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot3_node");
    ros::NodeHandle nh;

    ROS_INFO("robot3_node start");

    ros::Subscriber sub1 = nh.subscribe("Robot3/scan", 10, sensorCallback);
    ros::Subscriber sub2 = nh.subscribe("Robot13Point", 10, robot1Callback);
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("Robot3/cmd_vel", 10);
    ros::Publisher pub2 = nh.advertise<geometry_msgs::Point>("Robot31Point", 10);

    geometry_msgs::Point R31;

    ros::Rate rate(5);

    while (ros::ok()) {
        R31.x = r1.goal.first;
        R31.y = r1.goal.second;
        pub.publish(velOutput);
        pub2.publish(R31);
        ros::spinOnce();
        rate.sleep();
    }

    ros::spin();

    return 0;
}
