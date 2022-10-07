#include "sched_instance.h"

#ifdef INSTANCE
/* If success, return 0 */
int set_sched_instance(pid_t pid, int sched_instance)
{
    return syscall(500, pid, sched_instance);
}

/* If success, return instance number */
int get_sched_instance(pid_t pid)
{
    return syscall(501, pid);
}

#endif