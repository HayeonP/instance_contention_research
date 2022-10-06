#include <ros/ros.h>

#include <synthetic_task_generator.h>

int main(int argc, char* argv[])
{   
    std::string node_name("test");
    ros::init(argc, argv, node_name);

    SyntheticTaskGenerator synthetic_task_generator;
    synthetic_task_generator.run();

    return 0;
}