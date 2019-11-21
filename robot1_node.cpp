#include "robot_node.h"

class Robot     r2;     // Robot2 data
class Robot     r3;     // Robot3 data

// Goal<x,y>
vector<pair<double, double> > r_list;       // List of possible Robots position
vector<pair<double, double> > r2_list;      // List of possible Robot2 position
vector<pair<double, double> > r3_list;      // List of possible Robot3 position

// Goal<x,y>
vector<pair<double, double> > clustered;    // Clustered data

// Scan data converted to <x,y>
vector<pair<double,double> >  raw_data;
pair<double,double>           s;
pair<double,double>           f;
pair<double,double>           goal;

double  dist;
bool    start_flag = true;

void Association();
void Identification();

void sensorCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    int     total_deg   = msg->angle_max / msg->angle_increment;
    for (int i = 0; i < (total_deg+1); i++)
    // Coordination
    {
        if (msg->ranges[i] && msg->ranges[i] < 1.0) { // Leave 0~1.0m data
            s   = make_pair(-1 * msg->ranges[i] * sin(i*PI/180), msg->ranges[i] * cos(i*PI/180));
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
    if (r2.goal.first && r3.goal.first && clustered.size() > 0) Association();              // If the data already exist
    else if (!r2.goal.first && !r3.goal.first && clustered.size() >= 2) Identification();   // If the data is initialized
    cout << endl << "- final r2" << endl;
    cout << "(" << r2.goal.first << "," << r2.goal.second << ")\n";
    cout << endl << "- final r3" << endl;
    cout << "(" << r3.goal.first << "," << r3.goal.second << ")\n";
    if (!raw_data.empty()) raw_data.clear();
    if (!clustered.empty()) clustered.clear();
    if (!r_list.empty()) r_list.clear();
    if (!r2_list.empty()) r2_list.clear();
    if (!r3_list.empty()) r3_list.clear();
}

void Association()
// Update to the nearest value within 0.3m of existing value
{
    for (vector<int>::size_type i = 0; i < clustered.size(); i++) {
        double r2_dist = sqrt(pow(clustered[i].first-r2.goal.first, 2) +
                              pow(clustered[i].second-r2.goal.second, 2));
        double r3_dist = sqrt(pow(clustered[i].first-r3.goal.first, 2) +
                              pow(clustered[i].second-r3.goal.second, 2));
        if (min(r2_dist, r3_dist) == r2_dist) r2_list.push_back(clustered[i]);  // Detected Robot puts possibility closer
        else r3_list.push_back(clustered[i]);
    }
    if (!r2_list.empty()) {
        int index = GetMin(r2_list);
        r2.goal.first = r2.goal.first * Kl + r2_list[index].first * (1 - Kl);   // Smoothing filter
        r2.goal.second = r2.goal.second * Kl + r2_list[index].second * (1 - Kl);
    }
    if (!r3_list.empty()) {
        int index = GetMin(r3_list);
        r3.goal.first = r3.goal.first * Kl + r3_list[index].first * (1 - Kl);
        r3.goal.second = r3.goal.second * Kl + r3_list[index].second * (1 - Kl);
    }
}

void Identification()
// Index which is Robot2 and Robot3
{
    while (!clustered.empty()) {
        int index = GetMin(clustered);
        r_list.push_back(clustered[index]);
        clustered.erase(clustered.begin()+index);
    }
    if (r_list[0].second > 0.2 || r_list[1].second > 0.2) return;
    double  r2_theta = atan2(r_list[0].second, r_list[0].first);
    double  r3_theta = atan2(r_list[1].second, r_list[1].first);
    int     r2_index, r3_index;
    if (abs(r2_theta - r3_theta) < PI) {
        if (r2_theta < r3_theta) {      // The standard to classify is angle[rad]
            r2_index = 0;
            r3_index = 1;
        }
        else {
            r2_index = 1;
            r3_index = 0;
        }
    }
    else {
        if (r2_theta > r3_theta) {
            r2_index = 0;
            r3_index = 1;
        }
        else {
            r2_index = 1;
            r3_index = 0;
        }
    }
    r2.goal = r_list[r2_index];
    r3.goal = r_list[r3_index];
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot1_node");
    ros::NodeHandle nh;

    ROS_INFO("robot1_node start");

    ros::Subscriber sub = nh.subscribe("scan", 10, sensorCallback);
    ros::Publisher pub2 = nh.advertise<geometry_msgs::Point>("Robot12Point", 10);
    ros::Publisher pub3 = nh.advertise<geometry_msgs::Point>("Robot13Point", 10);

    ros::Rate rate(5);

    geometry_msgs::Point R12;
    geometry_msgs::Point R13;

    while (ros::ok()) {
        R12.x = r2.goal.first;
        R12.y = r2.goal.second;
        R13.x = r3.goal.first;
        R13.y = r3.goal.second;
        pub2.publish(R12);
        pub3.publish(R13);
        ros::spinOnce();
        rate.sleep();
    }

    ros::spin();

    return 0;
}
