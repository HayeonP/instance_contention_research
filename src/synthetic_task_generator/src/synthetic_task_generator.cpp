#include "synthetic_task_generator.h"

SyntheticTaskGenerator::SyntheticTaskGenerator()
    : private_nh_("~")
    , is_sync_(true)
    , instance_(0)
{
    /* Get rosparams */
    std::vector<std::string> pub_str_vec;
    std::vector<std::string> sub_str_vec;
    std::vector<std::string> sync_str_vec;

    node_name_ = ros::this_node::getName();

    if(!private_nh_.getParam("sub_list", sub_str_vec))
    {
        ROS_ERROR("Cannot find param: sub_list");
        exit(1);
    }

    if(!private_nh_.getParam("pub_list", pub_str_vec))
    {
        ROS_ERROR("Cannot find param: pub_list");
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
    private_nh_.param<bool>("instance_mode", instance_mode_, false);
    private_nh_.param<double>("rate", rate_, 10);
    private_nh_.param<double>("default_exec_time", default_exec_time_, 100.0);
    private_nh_.param<double>("callback_exec_time", callback_exec_time_, 100.0);
    

    /* Define Pub & Sub */
    if(sub_str_vec.size() == 0) is_source_ = true;

    if(!is_source_)
    {
        for(int i = 0; i < sub_str_vec.size(); i++){
            ros::Subscriber sub;
            // sub = nh_.subscribe(sub_str_vec[i].c_str(), 1, boost::bind(&SyntheticTaskGenerator::callback, _1, i), this);
            sub = nh_.subscribe<synthetic_task_generator::SyntheticTaskMsg>(sub_str_vec[i].c_str(), 1, boost::bind(&SyntheticTaskGenerator::callback, this, boost::placeholders::_1, i, sub_str_vec[i].c_str()));
            sub_vec_.push_back(sub);
        }
    }

    for(int i = 0; i < pub_str_vec.size(); i++){
        ros::Publisher pub;
        pub = nh_.advertise<synthetic_task_generator::SyntheticTaskMsg>(pub_str_vec[i].c_str(), 1);
        pub_vec_.push_back(pub);
    }    
    
    /* Define Sync */
    if(is_sync_){
        for(int i = 0; i < sub_str_vec.size(); i++){        
            bool need_sync = false;        
            for(int j = 0; j < sync_str_vec.size(); j++){
                if(sync_str_vec[j].compare(sub_str_vec[i]) == 0){ // Both are same
                    need_sync = true;
                    break;
                }                
            }
            
            if(need_sync) need_sync_vec_.push_back(true);        
            else need_sync_vec_.push_back(false);

            ready_to_sync_vec_.push_back(false);

        }

        int need_sync_cnt = 0;
        for(int i = 0; i < need_sync_vec_.size(); i++){
            if(need_sync_vec_[i]) need_sync_cnt++;
        }

        if(need_sync_cnt == 0){
            std::cout<<"[ERROR - "<< node_name_ <<"] sync topic doesn't exist in subscription topic list."<<std::endl;
            exit(1);
        }
    }

    /* Check instnace mode is enabled */
    #ifndef INSTANCE
    if(instance_mode_){
        std::cout<<"[ERROR - "<< node_name_ <<"] Cannot enable instance mode since pkg was built without INSTANCE macro."<<std::endl;
        exit(1);
    }
    #endif

    /* Print variables(debug) */
    if(debug_) print_variables(pub_str_vec, sub_str_vec, sync_str_vec);
}

SyntheticTaskGenerator::~SyntheticTaskGenerator()
{}

void SyntheticTaskGenerator::print_variables(std::vector<std::string> pub_str_vec, std::vector<std::string> sub_str_vec , std::vector<std::string> sync_str_vec)
{
    std::cout<<"====================================="<<std::endl;
    std::cout<<"- node_name: " << node_name_ <<std::endl;
    std::cout<<"- instance_mode: " << instance_mode_ <<std::endl;
    std::cout<<"- is_source: " << is_source_  <<std::endl;
    std::cout<<"- is_sync: " << is_sync_  <<std::endl;
    std::cout<<"- rate: " << rate_ <<std::endl;
    std::cout<<"- default_exec_time: " <<default_exec_time_  <<std::endl;
    std::cout<<"- callback_exec_time: " << callback_exec_time_  <<std::endl;
    
    std::cout<< "- Subscribe: " <<std::endl<<"\t[";
    if(sub_str_vec.size() > 1){
        std::cout<<sub_str_vec[0];
        for(auto it = sub_str_vec.begin()+1; it != sub_str_vec.end(); ++it){
            std::cout<< ", " << *it;
        }
    }
    std::cout<<"]"<<std::endl;

    std::cout<< "- Publish: " <<std::endl<<"\t[";
    if(pub_str_vec.size() > 1){
        std::cout<<pub_str_vec[0];    
        for(auto it = pub_str_vec.begin()+1; it != pub_str_vec.end(); ++it){
            std::cout<< ", " << *it;
        }
    }
    std::cout<<"]"<<std::endl;    

    std::cout<< "- Sync: " <<std::endl<<"\t[";
    if(sync_str_vec.size() > 1){
        std::cout<<sync_str_vec[0];
        for(auto it = sync_str_vec.begin()+1; it != sync_str_vec.end(); ++it){
            std::cout<< ", " << *it;
        }
    }
    std::cout<<"]"<<std::endl;

    std::cout<<"====================================="<<std::endl;
}

bool SyntheticTaskGenerator::is_ready_to_publish(){
    if(!is_sync_) return true;    

    for(int i = 0; i < need_sync_vec_.size(); i++){
        if(need_sync_vec_[i] == true && ready_to_sync_vec_[i] == false) return false;
    }

    return true;
}

void SyntheticTaskGenerator::run()
{   
    ros::Rate rate(rate_);

    while(ros::ok()){
        #ifdef INSTANCE
        if(instance_mode_) set_sched_instance(0, instance_);
        #endif

        ros::spinOnce();

        if(is_ready_to_publish()){
            for(int i = 0; i < pub_vec_.size(); i++)
            {            
                synthetic_task_generator::SyntheticTaskMsg msg;
                #ifdef INSTANCE
                if(instance_mode_) msg.instance = instance_;
                #endif
                msg.value = pub_data_vec_[i]++;
                pub_vec_[i].publish(msg);                                
            }

            for(int i = 0; i < ready_to_sync_vec_.size(); i++) ready_to_sync_vec_[i] = false;
        }
        

        rate.sleep();
    }
    return;
}

void SyntheticTaskGenerator::callback(synthetic_task_generator::SyntheticTaskMsgConstPtr msg, const int &sub_idx, const std::string topic_name)
{    
    if(debug_) std::cout<<"["<<node_name_<<"] callback for "<<topic_name<<" - Value: "<<msg->value<<std::endl;
    if(is_sync_ && need_sync_vec_[sub_idx]){
        ready_to_sync_vec_[sub_idx] = true;
    }

    #ifdef INSTANCE
    if(is_source_) instance_++;
    if(instance_mode_) instance_ = msg->instance;
    #endif

    return;
}