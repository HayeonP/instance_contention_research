#include "synthetic_task_generator.h"


std::ofstream response_time_log_file_;
std::ofstream time_log_file_;

SyntheticTaskGenerator::SyntheticTaskGenerator()
    : private_nh_("~")
    , is_sync_(true)
    , instance_(0)
    ,is_ready_to_set_schd_instance_(false)
    ,is_job_finished_(false)
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
    private_nh_.param<bool>("is_source", is_source_, false);
    private_nh_.param<std::string>("sched_policy", sched_policy_, std::string("NONE"));
    private_nh_.param<int>("sched_priority", sched_priority_, 0);
    private_nh_.param<bool>("instance_mode", instance_mode_, false);
    private_nh_.param<double>("rate", rate_, 10);
    private_nh_.param<double>("default_exec_time", default_exec_time_, 100.0);
    private_nh_.param<double>("callback_exec_time", callback_exec_time_, 100.0);
    private_nh_.param<int>("cur_pid_list_size", cur_pid_list_size_, 0);
    private_nh_.param<int>("next_pid_list_size", next_pid_list_size_, 0);
    
    /* Scheduyling Setup */
    if(sched_policy_ != std::string("CFS") && sched_policy_ != std::string("FIFO")){
        std::cout<<"## "<<sched_policy_<<std::endl;
        std::cout << "[ERROR] Please use CFS or FIFO for scheduling policy." << std::endl;
        exit(1);
    }
    if(sched_priority_ < 0 || sched_priority_ > 99){
        std::cout << "[ERROR] Sched priority for SCHED_FIFO should be [0, 99]." << std::endl;
        exit(1);
    }

    /* Define Pub & Sub */    
    for(int i = 0; i < sub_str_vec.size(); i++){
        ros::Subscriber sub;
        // sub = nh_.subscribe(sub_str_vec[i].c_str(), 1, boost::bind(&SyntheticTaskGenerator::callback, _1, i), this);
        sub = nh_.subscribe<synthetic_task_generator::SyntheticTaskMsg>(sub_str_vec[i].c_str(), 1, boost::bind(&SyntheticTaskGenerator::callback, this, boost::placeholders::_1, i, sub_str_vec[i].c_str()));
        sub_vec_.push_back(sub);
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
    else if(sub_str_vec.size() == 1) std::cout<<sub_str_vec[0];
    std::cout<<"]"<<std::endl;

    std::cout<< "- Publish: " <<std::endl<<"\t[";
    if(pub_str_vec.size() > 1){
        std::cout<<pub_str_vec[0];    
        for(auto it = pub_str_vec.begin()+1; it != pub_str_vec.end(); ++it){
            std::cout<< ", " << *it;
        }
    }
    else if(pub_str_vec.size() == 1) std::cout<<pub_str_vec[0];
    std::cout<<"]"<<std::endl;    

    std::cout<< "- Sync: " <<std::endl<<"\t[";
    if(sync_str_vec.size() > 1){
        std::cout<<sync_str_vec[0];
        for(auto it = sync_str_vec.begin()+1; it != sync_str_vec.end(); ++it){
            std::cout<< ", " << *it;
        }
    }
    else if(sync_str_vec.size() == 1) std::cout<<sync_str_vec[0];
    std::cout<<"]"<<std::endl;

    std::cout<<"====================================="<<std::endl;
}

bool SyntheticTaskGenerator::is_ready_to_publish(){
    if(is_source_) return true;
    if(!is_job_finished_) return false;

    for(int i = 0; i < need_sync_vec_.size(); i++){
        if(need_sync_vec_[i] == true && ready_to_sync_vec_[i] == false) return false;
    }

    return true;
}

void SyntheticTaskGenerator::run()
{   
    ros::Rate rate(rate_);
    timer_init(4);
    timer_reset(0);

    double max = 0;
    int cold_start_cnt = 0;

    response_time_log_file_.open(std::string("/home/nvidia/git/instance_contention_research/log/response_time/")+node_name_.substr(1)+std::string(".csv"));
    response_time_log_file_ << "PID,start,end,instance\n";
    
    if(debug_){        
        time_log_file_.open(std::string("/home/nvidia/git/instance_contention_research/log/time_log_")+node_name_.substr(1)+std::string(".csv"));
        time_log_file_ << "total,update_instance,default,spin,set_instance\n";
    }
    
    if(is_source_){
        while(ros::ok()){
            std::string start_time, end_time;
            start_time = get_current_time();
            if(is_ready_to_set_schd_instance_ == true) timer_start(0);     
    
            #ifdef INSTANCE
            if(instance_mode_){
                if(!is_ready_to_set_schd_instance_){
                    bool is_ready_to_set_schd_instance_param = false;
                    std::vector<std::string> cur_pid_str_vec, next_pid_str_vec;
    
                    private_nh_.getParam("cur_pid_list_size", cur_pid_list_size_);
                    private_nh_.getParam("next_pid_list_size", next_pid_list_size_);
                    private_nh_.getParam("is_ready_to_set_schd_instance", is_ready_to_set_schd_instance_param);
                    private_nh_.getParam("cur_pid_list", cur_pid_str_vec);
                    private_nh_.getParam("next_pid_list", next_pid_str_vec);
    
                    if(is_ready_to_set_schd_instance_param == true && cur_pid_list_size_ > 0 && cur_pid_str_vec.size() == cur_pid_list_size_ && next_pid_str_vec.size() == next_pid_list_size_)
                    {
                        cur_pid_vec_.clear();
                        next_pid_vec_.clear();
                        for(int i = 0; i < cur_pid_list_size_; i++) cur_pid_vec_.push_back(std::stoi(cur_pid_str_vec[i]));
                        for(int i = 0; i < next_pid_list_size_; i++) next_pid_vec_.push_back(std::stoi(next_pid_str_vec[i]));
                        is_ready_to_set_schd_instance_ = true;
    
                        /* Setup RT scheduler and priority */
                        if(sched_policy_ == std::string("FIFO")){
                            for(int i = 0; i < cur_pid_vec_.size(); i++){
                                struct sched_param sp = { .sched_priority = sched_priority_ };
                                if(i != 0) sp.sched_priority = 99; // Non-main thread get high priority                  
                                if (sched_setscheduler(cur_pid_vec_[i], SCHED_FIFO, &sp) < 0) {
                                    perror("sched_setscheduler");
                                    exit(1);
                                }
                            }
                        }
                    }
                }
    
                if(is_source_){                
                    instance_ = instance_ + 1;
                    for(int i = 0; i < cur_pid_vec_.size(); i++) set_sched_instance((pid_t)(cur_pid_vec_[i]), instance_);
                }
    
                for(int i = 0; i < cur_pid_vec_.size(); i++) update_sched_instance((pid_t)(cur_pid_vec_[i]));                
    
                if(!is_source_) instance_ = get_sched_instance(0);
    
            }
            #endif
            if(is_ready_to_set_schd_instance_ == true) timer_stop(0);
    
            timer_start(1);
            for(unsigned long long cnt = 0; cnt < MS_CNT(default_exec_time_); cnt++) int a = (cnt + 10) * 2000 / 127;
            timer_stop(1);
    
            timer_start(2);
            ros::spinOnce();
            timer_stop(2);
    
            timer_start(3);
            if(is_ready_to_publish()){
                /* Finish job */
                #ifdef INSTANCE
                if(instance_mode_){
                    for(int i = 0; i < next_pid_vec_.size(); i++) set_sched_instance((pid_t)(next_pid_vec_[i]), instance_); // *it: next pid
                }
                #endif
    
                for(int i = 0; i < pub_vec_.size(); i++)
                {            
                    synthetic_task_generator::SyntheticTaskMsg msg;
                    msg.instance = instance_;
                    msg.value = pub_data_vec_[i]++;
                    pub_vec_[i].publish(msg);                                
                }
    
                for(int i = 0; i < ready_to_sync_vec_.size(); i++) ready_to_sync_vec_[i] = false;                
            }
            timer_stop(3);
    
            if(debug_){
                time_log_file_ << timer_read(0) + timer_read(1) + timer_read(2) << "," << timer_read(0) << "," << timer_read(1) << "," <<timer_read(2)<<","<<timer_read(3)<<"\n";
                time_log_file_.flush();
            }        
            timer_reset(0); timer_reset(1); timer_reset(2); timer_reset(3);
    
            end_time = get_current_time();
    
            response_time_log_file_ << getpid() << "," << start_time << "," << end_time << "," << instance_ << std::endl;
    
            if(is_job_finished_ || is_source_){
                is_job_finished_ = false;
                debug_finish_job(0);
            }
            
            rate.sleep();
        }
    }
    else{
        ros::spin();
    }


    return;
}

void SyntheticTaskGenerator::callback(synthetic_task_generator::SyntheticTaskMsgConstPtr msg, const int &sub_idx, const std::string topic_name)
{  
    std::string start_time, end_time;
    start_time = get_current_time();
    if(is_ready_to_set_schd_instance_ == true) timer_start(0);

    /* Callback */
    if(is_sync_ && need_sync_vec_[sub_idx]){
        ready_to_sync_vec_[sub_idx] = true;
    }

    for(unsigned long long cnt = 0; cnt < MS_CNT(callback_exec_time_); cnt++) int a = (cnt + 10) * 2000 / 127;

    #ifdef INSTANCE
    if(is_source_) instance_++;
    if(instance_mode_) instance_ = msg->instance;
    #endif

    is_job_finished_ = true;

    /* Start job */
    #ifdef INSTANCE
    if(instance_mode_){
        if(!is_ready_to_set_schd_instance_){
            bool is_ready_to_set_schd_instance_param = false;
            std::vector<std::string> cur_pid_str_vec, next_pid_str_vec;

            private_nh_.getParam("cur_pid_list_size", cur_pid_list_size_);
            private_nh_.getParam("next_pid_list_size", next_pid_list_size_);
            private_nh_.getParam("is_ready_to_set_schd_instance", is_ready_to_set_schd_instance_param);
            private_nh_.getParam("cur_pid_list", cur_pid_str_vec);
            private_nh_.getParam("next_pid_list", next_pid_str_vec);

            if(is_ready_to_set_schd_instance_param == true && cur_pid_list_size_ > 0 && cur_pid_str_vec.size() == cur_pid_list_size_ && next_pid_str_vec.size() == next_pid_list_size_)
            {
                cur_pid_vec_.clear();
                next_pid_vec_.clear();
                for(int i = 0; i < cur_pid_list_size_; i++) cur_pid_vec_.push_back(std::stoi(cur_pid_str_vec[i]));
                for(int i = 0; i < next_pid_list_size_; i++) next_pid_vec_.push_back(std::stoi(next_pid_str_vec[i]));
                is_ready_to_set_schd_instance_ = true;

                /* Setup RT scheduler and priority */
                if(sched_policy_ == std::string("FIFO")){
                    for(int i = 0; i < cur_pid_vec_.size(); i++){
                        struct sched_param sp = { .sched_priority = sched_priority_ };
                        if(i != 0) sp.sched_priority = 99; // Non-main thread get high priority                  
                        if (sched_setscheduler(cur_pid_vec_[i], SCHED_FIFO, &sp) < 0) {
                            perror("sched_setscheduler");
                            exit(1);
                        }
                    }
                }
            }
        }

        for(int i = 0; i < cur_pid_vec_.size(); i++) update_sched_instance((pid_t)(cur_pid_vec_[i]));                
        
        if(!is_source_) instance_ = get_sched_instance(0);
        
    }
    #endif
    if(is_ready_to_set_schd_instance_ == true) timer_stop(0);

    timer_start(1);
    for(unsigned long long cnt = 0; cnt < MS_CNT(default_exec_time_); cnt++) int a = (cnt + 10) * 2000 / 127;
    timer_stop(1);

    timer_start(2);
    timer_stop(2);

    timer_start(3);
    if(is_ready_to_publish()){
        /* Finish job */
        #ifdef INSTANCE
        if(instance_mode_){
            for(int i = 0; i < next_pid_vec_.size(); i++) set_sched_instance((pid_t)(next_pid_vec_[i]), instance_); // *it: next pid
        }
        #endif

        for(int i = 0; i < pub_vec_.size(); i++)
        {            
            synthetic_task_generator::SyntheticTaskMsg msg;
            msg.instance = instance_;
            msg.value = pub_data_vec_[i]++;
            pub_vec_[i].publish(msg);                                
        }

        for(int i = 0; i < ready_to_sync_vec_.size(); i++) ready_to_sync_vec_[i] = false;                
    }
    timer_stop(3);

    if(debug_){
        time_log_file_ << timer_read(0) + timer_read(1) + timer_read(2) << "," << timer_read(0) << "," << timer_read(1) << "," <<timer_read(2)<<","<<timer_read(3)<<"\n";
        time_log_file_.flush();
    }        
    timer_reset(0); timer_reset(1); timer_reset(2); timer_reset(3);

    end_time = get_current_time();
    
    response_time_log_file_ << getpid() << "," << start_time << "," << end_time << "," << instance_ << std::endl;

    if(is_job_finished_ || is_source_){
        is_job_finished_ = false;
        debug_finish_job(0);
    }

    return;
}
