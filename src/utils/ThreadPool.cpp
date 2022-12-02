//Copyright (c) 2022 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "utils/ThreadPool.h"

namespace cura
{

ThreadPool::ThreadPool(size_t nthreads)
  : wait_for_new_tasks(true)
{
    for (size_t i = 0 ; i < nthreads; i++)
    {
        threads.emplace_back(&ThreadPool::worker, this);
    }
}

void ThreadPool::worker()
{
    lock_t lock = get_lock();
    work_while(lock, [this, &lock]()
        {
            while(tasks.empty() && wait_for_new_tasks)
            {  // Wait for a task. Signaled by ThreadPool::push() and ThreadPool::join()
               condition.wait(lock);
            }
            // Returns false if the queue is empty and the pool is being disposed
            return !tasks.empty() || wait_for_new_tasks;
        });
}

void ThreadPool::join()
{
    { // Joinning thread becomes a worker while there is remaining tasks
        lock_t lock = get_lock();
        wait_for_new_tasks = false;
        condition.notify_all();
        work_while(lock, []{ return true; });
    }
    assert(tasks.empty());
    for (auto& thread : threads)
    {
        thread.join();
    }
    threads.clear();
}

} //Cura namespace.
