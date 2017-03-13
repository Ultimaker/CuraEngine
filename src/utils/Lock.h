/** Copyright (C) 2017 Ultimaker - Released under terms of the AGPLv3 License */
#ifndef UTILS_LOCK_H
#define UTILS_LOCK_H


#ifdef _OPENMP
    #include <omp.h>
#endif // _OPENMP


class Lock
{
public:
    Lock()
    {
#ifdef _OPENMP
        omp_init_lock(&lock_object);
#endif // _OPENMP
    }
    ~Lock()
    {
#ifdef _OPENMP
        omp_destroy_lock(&lock_object);
#endif // _OPENMP
    }
    void lock()
    {
#ifdef _OPENMP
        omp_set_lock(&lock_object);
#endif // _OPENMP
    }
    void unlock()
    {
#ifdef _OPENMP
        omp_unset_lock(&lock_object);
#endif // _OPENMP
    }
    int test_lock()
    {
        int ret = 1;
#ifdef _OPENMP
        ret = omp_test_lock(&lock_object);
#endif // _OPENMP
        return ret;
    }
private:
#ifdef _OPENMP
    omp_lock_t lock_object;
#endif // _OPENMP
    Lock(const Lock&) = delete;
    Lock& operator=(const Lock&) = delete;
};

#endif // UTILS_MULTITHREADING_LOCK_H