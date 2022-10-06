#ifndef SYNTHETIC_TASK_GENERATOR_H
#define SYNTHETIC_TASK_GENERATOR_H

// C++ includes
#include <string>
#include <yaml-cpp/yaml.h>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

class SyntheticTaskGenerator{
public:
    SyntheticTaskGenerator();    
    ~SyntheticTaskGenerator();
    void run();

private: // Functions
    void initForROS();
    void print_variables(std::vector<std::string> pub_str_vec, std::vector<std::string> sub_str_vec , std::vector<std::string> sync_str_vec);
    void callback(const geometry_msgs::PoseStamped &msg);
private: // Variables
    
    /* Handle */
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    /* Pub & Sub */
    std::vector<ros::Publisher> pub_vec_;    
    std::vector<ros::Subscriber> sub_vec_;
    std::vector<std::string> sync_vec_;
    std::vector<double> pub_data_vec_;
    
    /* Task parameters */
    double rate_, default_exec_time_, callback_exec_time_; //ms

    /* Others */    
    std::string node_name_;
    bool is_source_, is_sync_, debug_;
};

#endif 
