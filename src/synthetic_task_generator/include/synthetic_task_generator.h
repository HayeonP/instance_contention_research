#ifndef SYNTHETIC_TASK_GENERATOR_H
#define SYNTHETIC_TASK_GENERATOR_H

// C++ includes
#include <string>
#include <yaml-cpp/yaml.h>

#include <ros/ros.h>
#include <synthetic_task_generator/SyntheticTaskMsg.h>

#ifdef INSTANCE
#include "sched_instance.h"
#endif

class SyntheticTaskGenerator{
public:
    SyntheticTaskGenerator();    
    ~SyntheticTaskGenerator();
    void run();
    
    void callback(synthetic_task_generator::SyntheticTaskMsgConstPtr msg, const int& sub_idx, const std::string topic_name);
private: // Functions
    void initForROS();
    void print_variables(std::vector<std::string> pub_str_vec, std::vector<std::string> sub_str_vec , std::vector<std::string> sync_str_vec);    
    bool is_ready_to_publish();
private: // Variables
    
    /* Handle */
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    /* Pub & Sub */
    std::vector<ros::Subscriber> sub_vec_;
    std::vector<ros::Publisher> pub_vec_;        
    std::vector<double> pub_data_vec_;

    /* Sync */
    std::vector<bool> need_sync_vec_;
    std::vector<bool> ready_to_sync_vec_;
    
    
    /* Task parameters */
    double rate_, default_exec_time_, callback_exec_time_; //ms

    /* Others */    
    int instance_;
    std::string node_name_;
    bool is_source_, is_sync_, instance_mode_, debug_;
};

#endif 
