#ifndef SYNTHETIC_TASK_GENERATOR_SCHED_INSTANCE_H
#define SYNTHETIC_TASK_GENERATOR_SCHED_INSTANCE_H

#ifdef INSTANCE
#include <linux/kernel.h>
#include <sys/syscall.h>
#include <unistd.h>
int set_sched_instance(pid_t pid, int sched_instance);
int get_sched_instance(pid_t pid);
int update_sched_instance(pid_t pid);
int debug_finish_job(pid_t pid);
#endif

#endif