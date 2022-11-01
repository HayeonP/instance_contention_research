import math
import json

class Task:
    def __init__(self, name, T_D, T_K, E_D, E_K, psi_P, psi_S, pri):
        self.name = name
        self.T_D = float(T_D)
        self.T_K = float(T_K)
        self.E_D = float(E_D)
        self.E_K = float(E_K)
        self.psi_P = psi_P
        self.psi_S = psi_S
        self.pri = pri
    
    def print(self):
        print('['+self.name+']')
        print('  - T_D:',self.T_D)
        print('  - T_D:', self.T_D)
        print('  - T_K:', self.T_K)
        print('  - E_D:', self.E_D)
        print('  - E_K:', self.E_K)
        print('  - psi_P:', self.psi_P)
        print('  - psi_S:', self.psi_S)
        print('  - pri:', self.pri)

def RTA_single_core(taskset):
    response_times = {}

    for i in taskset:
        R = 0.0
        task_i = taskset[i]
        for j in taskset:
            task_j = taskset[j]
            if task_j.pri < task_i.pri: continue
            R = R + task_j.E_D + task_j.E_K
        R_prev = R        

        for k in range(10):
            R = task_i.E_D + task_i.E_K

            for j in taskset:
                task_j = taskset[j]
                if task_j.pri <= task_i.pri: continue            
                R = R + math.ceil(R_prev/task_j.T_D) * task_j.E_D + math.ceil(R_prev/task_j.T_K) * task_j.E_K

            if R_prev == R: break
            R_prev = R

        response_times[task_i.name] = R
            
    return response_times

if __name__ == '__main__':
    taskset = {}
    with open('./configs/221101_feasible_single_core_taskset.json') as f:
        raw_data = json.load(f)
        for task_name in raw_data:
            task_info = raw_data[task_name]
            taskset[task_name] = Task(task_name, task_info['T_D'], task_info['T_K'], task_info['E_D'], task_info['E_K'], task_info['psi_P'], task_info['psi_S'], task_info['pri'])
    
    
    response_times = RTA_single_core(taskset)
    print(response_times)

