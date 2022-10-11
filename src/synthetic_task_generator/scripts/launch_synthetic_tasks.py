import rospy
import json
import os
import subprocess
import signal, sys, time

config_options = ['debug', 'instance_mode', 'rate', 'default_exec_time', 'callback_exec_time', 'sub_list', 'pub_list', 'pub_data', 'sync_list', 'next_list']

pid_info = {}

def terminate_synthetic_tasks():
    for node_name in pid_info:
        for pid in pid_info[node_name]:
            os.system('kill -9 '+pid)

def signal_handler(sig, frame):
    terminate_synthetic_tasks()

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
        self.next_list = []
    
    def update(self, json_config):
        if not self.validation (json_config):
            return False
        
        self.debug = json_config['debug']
        self.instance_mode = json_config['instance_mode']
        self.rate = json_config['rate']
        self.default_exec_time = json_config['default_exec_time']
        self.callback_exec_time = json_config['callback_exec_time']
        self.pub_list = json_config['pub_list']
        self.pub_data = json_config['pub_data']
        self.sub_list = json_config['sub_list']
        self.sync_list = json_config['sync_list']
        self.next_list = json_config['next_list']

        return True

    def validation(self, json_config):    
        for config_option in config_options:
            if config_option not in json_config:     
                print("[ERROR] "+self.name+" needs "+config_option+".")           
                return False

        if len (json_config['pub_list']) != len (json_config['pub_data']):
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
        print('\t - next_list:', self.next_list)
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

    def set_rosparam_next_list(self):
        next_list_param = []
        for next in self.next_list:
            for pid in pid_info[next]: next_list_param.append(pid)
        print(next_list_param)

        rospy.set_param('/'+self.name+'/next_list', next_list_param)


def create_launch_script(path, json_config_list):
    with open(path, 'w') as f:
        f.write('<launch>\n')

        for json_config in json_config_list:
            name = json_config.name
            f.write('\t<node pkg=\"synthetic_task_generator\" type=\"synthetic_task_generator\" name=\"' + name + '\" output=\"screen\"/>\n')

        f.write('</launch>')

def get_pid(name):
    return subprocess.check_output(["pidof",name])

def update_pid_info (json_config_list):
    for node_name in json_config_list:
        output = os.popen('ps -ef -T | grep "synthetic_task_generator __name:="'+ node_name)
        pid_list = []
        for line in output:
            if 'grep' in line: continue                  
            pid = line.split()[2]
            pid_list.append(pid)
        pid_info[node_name] = pid_list             
    return

def validate_next_pids_are_prepared(json_config_list, node_config_list):
    for node_config in node_config_list:        
        for next in node_config.next_list:
            if next not in json_config_list:
                print('[ERROR - '+node_config.name+'] Next node name \"'+next+'\" does not exist.')
                terminate_synthetic_tasks()
                exit(1)
            if next not in pid_info: return False
            if len(pid_info[next]) < 5: return False

    return True


def main():
    signal.signal(signal.SIGINT, signal_handler)

    json_config_list = {}
    with open('./configs.json') as f:
        json_config_list = json.load(f)

    node_config_list = []
    for node_name in json_config_list:
        node_config = NodeConfig(node_name)
        if not node_config.update (json_config_list[node_name]): exit(1)
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
            if not is_pid_info_initialized:
                update_pid_info (json_config_list)
                is_pid_info_initialized = validate_next_pids_are_prepared(json_config_list, node_config_list)
                if is_pid_info_initialized:
                    for node_config in node_config_list: node_config.set_rosparam_next_list()
                        
                

    return

if __name__ == "__main__":
    main()