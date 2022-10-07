import rospy
import json
import os
import subprocess
import signal, sys, time

config_options = ['debug', 'rate', 'default_exec_time', 'callback_exec_time', 'pub_list', 'sub_list', 'sync_list']

pid_info = {}

def signal_handler(sig, frame):
    for node_name in pid_info:
        for pid in pid_info[node_name]:
            os.system('kill -9 '+pid)

    sys.exit()

class NodeConfig:
    def __init__(self, name):
        self.name = name
        self.debug = False
        self.instance_mode = False
        self.rate = -1.0
        self.default_exec_time = -1.0
        self.callback_exec_time = -1.0
        self.pub_list = []
        self.pub_data = []
        self.sub_list = []
        self.sync_list = []
    
    def update(self, config):
        if not self.validation(config):
            return False
        
        self.debug = config['debug']
        self.instance_mode = config['instance_mode']
        self.rate = config['rate']
        self.default_exec_time = config['default_exec_time']
        self.callback_exec_time = config['callback_exec_time']
        self.pub_list = config['pub_list']
        self.pub_data = config['pub_data']
        self.sub_list = config['sub_list']
        self.sync_list = config['sync_list']

        return True

    def validation(self, config):    
        for config_option in config_options:
            if config_option not in config:     
                print("[ERROR] "+self.name+" needs "+config_option+".")           
                return False

        if len(config['pub_list']) != len(config['pub_data']):
            print("[ERROR] Length of pub_list and pub_data for node \""+self.name+"\" should be same.")
            return False

        return True

    def print(self):        

        print("=================================")
        print(" - name: " + self.name)
        print('\t - debug: ' + str(self.debug))
        print('\t - instance_mode: ' + str(self.instance_mode))
        print('\t - rate: ' + str(self.rate))
        print('\t - default_exec_time: ' + str(self.default_exec_time))
        print('\t - callback_exec_time: ' + str(self.callback_exec_time))
        print('\t - pub_list:', self.pub_list)
        print('\t - pub_data:', self.pub_data)
        print('\t - sub_list:', self.sub_list)
        print('\t - sync_list:', self.sync_list)
        print("=================================")
        
        return

    def set_rosparam(self):

        rospy.set_param('/'+self.name+'/debug', self.debug)
        rospy.set_param('/'+self.name+'/instance_mode', self.instance_mode)
        rospy.set_param('/'+self.name+'/rate', self.rate)
        rospy.set_param('/'+self.name+'/default_exec_time', self.default_exec_time)
        rospy.set_param('/'+self.name+'/callback_exec_time', self.callback_exec_time)
        rospy.set_param('/'+self.name+'/pub_list', self.pub_list)
        rospy.set_param('/'+self.name+'/pub_data', self.pub_data)
        rospy.set_param('/'+self.name+'/sub_list', self.sub_list)
        rospy.set_param('/'+self.name+'/sync_list', self.sync_list)

        return

def create_launch_script(path, config_list):
    with open(path, 'w') as f:
        f.write('<launch>\n')

        for config in config_list:
            name = config.name
            f.write('\t<node pkg=\"synthetic_task_generator\" type=\"synthetic_task_generator\" name=\"' + name + '\" output=\"screen\"/>\n')

        f.write('</launch>')

def get_pid(name):
    return subprocess.check_output(["pidof",name])

def update_pid_info(configs):
    for node_name in configs:
                output = os.popen('ps -ef -T | grep "synthetic_task_generator __name:="'+ node_name)
                pid_list = []
                for line in output:
                    if 'grep' in line: continue                  
                    pid = line.split()[2]
                    pid_list.append(pid)
                pid_info['node_name'] = pid_list             
    return

def main():
    signal.signal(signal.SIGINT, signal_handler)

    configs = {}
    with open('./configs.json') as f:
        configs = json.load(f)

    node_config_list = []
    for node_name in configs:
        node_config = NodeConfig(node_name)
        if not node_config.update(configs[node_name]): exit(1)
        node_config_list.append(node_config)
        os.system('rosparam delete /'+node_name)

    launch_script_path = '/home/hypark/git/instance_contention_research/src/synthetic_task_generator/launch/synthetic_task_generator.launch'

    for node_config in node_config_list:
        node_config.print()
        node_config.set_rosparam()
    
    create_launch_script(launch_script_path, node_config_list)

    if os.fork() == 0: # Child process
        # Launch node
        os.system('roslaunch synthetic_task_generator synthetic_task_generator.launch')
    else:
        is_pid_info_initialized = False
        while(1):
            time.sleep(3)            
            update_pid_info(configs)
            if not is_pid_info_initialized:
                is_pid_info_initialized = True
                print("[INFO] PID info is initialized")
    return

if __name__ == "__main__":
    main()