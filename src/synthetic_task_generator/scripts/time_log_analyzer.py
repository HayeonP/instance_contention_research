import csv

path = '/home/nvidia/git/instance_contention_research/log/time_log_test3.csv'

total_list = []
update_instance_list = []
default_list = []
spin_list = []
set_instance_list = []

with open(path) as f:
    reader = csv.reader(f)
    for line in reader:
        if 'total' in line: continue
        total_list.append(float(line[0]))
        update_instance_list.append(float(line[1]))
        default_list.append(float(line[2]))
        spin_list.append(float(line[3]))
        set_instance_list.append(float(line[4]))
    

print('len:',len(total_list))
print('total_list(max/min):', max(total_list), '/', sum(total_list)/len(total_list))
print('update_instance_list(max/min):', max(update_instance_list), '/', sum(update_instance_list)/len(update_instance_list))
print('default_list(max/min):', max(default_list), '/', sum(default_list)/len(default_list))
print('spin_list(max/min):', max(spin_list), '/', sum(spin_list)/len(spin_list))
print('set_instance_list(max/min):', max(set_instance_list), '/', sum(set_instance_list)/len(set_instance_list))