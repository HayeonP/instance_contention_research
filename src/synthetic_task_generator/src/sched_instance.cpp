#include "sched_instance.h"

#ifdef INSTANCE
/* If success, return 0 */
int set_sched_instance(pid_t pid, int sched_instance)
{
    return syscall(291, pid, sched_instance);
}

/* If success, return instance number */
int get_sched_instance(pid_t pid)
{
    return syscall(292, pid);
}

/* If success, return 0 */
int update_sched_instance(pid_t pid)
{
    return syscall(293, pid);
}

int debug_finish_job(pid_t pid)
{
    return syscall(294, pid);
}

#endif