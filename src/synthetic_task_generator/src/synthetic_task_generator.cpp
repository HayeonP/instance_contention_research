#include "synthetic_task_generator.h"

SyntheticTaskGenerator::SyntheticTaskGenerator()
    : private_nh_("~")
    , is_sync_(true)
{
    /* Get rosparams */
    std::vector<std::string> pub_str_vec;
    std::vector<std::string> sub_str_vec;
    std::vector<std::string> sync_str_vec;

    node_name_ = ros::this_node::getName();

    if(!private_nh_.getParam("pub_list", pub_str_vec))
    {
        ROS_ERROR("Cannot find param: pub_list");
        exit(1);
    }

    if(!private_nh_.getParam("sub_list", sub_str_vec))
    {
        ROS_ERROR("Cannot find param: sub_list");
        exit(1);
    }

    if(!private_nh_.getParam("sync_list", sync_str_vec))
    {
        ROS_ERROR("Cannot find param: sync_list");
        exit(1);
    }

    if(!private_nh_.getParam("pub_data", pub_data_vec_))
    {
        ROS_ERROR("Cannot find param: pub_data");
        exit(1);
    }


    if(sync_str_vec.empty()) is_sync_ = false;

    private_nh_.param<bool>("debug", debug_, false);
    private_nh_.param<double>("rate", rate_, 10);
    private_nh_.param<double>("default_exec_time", default_exec_time_, 100.0);
    private_nh_.param<double>("callback_exec_time", callback_exec_time_, 100.0);
    

    /* Define Pub & Sub */
    for(int i = 0; i < pub_str_vec.size(); i++){
        ros::Publisher pub;
        pub = nh_.advertise<geometry_msgs::PoseStamped>(pub_str_vec[i].c_str(), 1);
        pub_vec_.push_back(pub);
    }

    if(sub_str_vec.size() == 0) is_source_ = true;

    if(!is_source_)
    {
        for(int i = 0; i < sub_str_vec.size(); i++){
            ros::Subscriber sub;
            sub = nh_.subscribe(sub_str_vec[i].c_str(), 1, &SyntheticTaskGenerator::callback ,this);
            sub_vec_.push_back(sub);
        }
    }

    if(debug_) print_variables(pub_str_vec, sub_str_vec, sync_str_vec);
    
}

SyntheticTaskGenerator::~SyntheticTaskGenerator()
{}

void SyntheticTaskGenerator::print_variables(std::vector<std::string> pub_str_vec, std::vector<std::string> sub_str_vec , std::vector<std::string> sync_str_vec)
{
    std::cout<<"====================================="<<std::endl;
    std::cout<<"- node_name: " << node_name_ <<std::endl;
    std::cout<<"- is_source: " << is_source_  <<std::endl;
    std::cout<<"- is_sync: " << is_sync_  <<std::endl;
    std::cout<<"- rate: " << rate_ <<std::endl;
    std::cout<<"- default_exec_time: " <<default_exec_time_  <<std::endl;
    std::cout<<"- callback_exec_time: " << callback_exec_time_  <<std::endl;
    
    std::cout<< "- Publish: " <<std::endl<<"\t["<<pub_str_vec[0];
    for(auto it = pub_str_vec.begin()+1; it != pub_str_vec.end(); ++it){
        std::cout<< ", " << *it;
    }
    std::cout<<"]"<<std::endl;

    std::cout<< "- Subscribe: " <<std::endl<<"\t["<<sub_str_vec[0];
    for(auto it = sub_str_vec.begin()+1; it != sub_str_vec.end(); ++it){
        std::cout<< ", " << *it;
    }
    std::cout<<"]"<<std::endl;

    std::cout<< "- Sync: " <<std::endl<<"\t["<<sync_str_vec[0];
    for(auto it = sync_str_vec.begin()+1; it != sync_str_vec.end(); ++it){
        std::cout<< ", " << *it;
    }
    std::cout<<"]"<<std::endl;

    std::cout<<"====================================="<<std::endl;
}

void SyntheticTaskGenerator::run()
{   
    ros::Rate rate(rate_);

    while(ros::ok()){
        ros::spinOnce();
        for(int i = 0; i < pub_vec_.size(); i++)
        {            
            geometry_msgs::PoseStamped msg;
            msg.pose.position.x = pub_data_vec_[i];
            pub_vec_[i].publish(msg);
        }

        rate.sleep();
    }
    return;
}

void SyntheticTaskGenerator::callback(const geometry_msgs::PoseStamped &msg)
{
    return;
}