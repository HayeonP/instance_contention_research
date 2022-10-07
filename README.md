# instance_contention_research

##  synthetic_task_generator
- Create synthetic ros task with configuration. All topic types are `geometry_msgs/PoseStamed`.
    ### How to use
    (1) Build
    ```
    cd $WORKSPACE_DIR
    catkin_make
    ```
    - If you need instance mode, enable macro in `src/synthetic_task_generator/CMakeLists.txt`
    ```
    # Enable following line if you need instance mode
    add_compile_definitions(INSTANCE)

    ```

    (2) Setup node configuration to `src/synthetic_task_generator/scrtips/configs.json`
    ```
    "node_name": {                          # Name for ROS node..
        "debug": false,                     # For visualize debug message.
        "instance_mode: false,              # Set instasnce mode
        "rate": 10.0,                       # ROS rate.
        "default_exec_time": 100.0,         # Execution time for main routine.
        "callback_exec_time": 100.0,        # Execution time for callback routine.
        "pub_list": ["topic1", "topic2"],   # List of topics for publish.
        "pub_data": [100, 200],             # List of data for publish message. Data will be written to msg.pose.position.x.
        "sub_list": ["topic0"],             # List of topic for subscribe.
        "sync_list": ["topic5]              # List of sync topic. Topic in pub list is published when all sync topic is subscribed.
    ```
    
    (3) Launch `launch_synthetic_tasks.py`
    ```
    python3 src/scripts/synthetic_task_generator/launch_synthetic_tasks.py
    ```