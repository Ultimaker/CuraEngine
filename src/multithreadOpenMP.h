/** Copyright (C) 2016 Ultimaker - Released under terms of the AGPLv3 License */

#ifndef MULTITHREAD_OPENMP_H
#define MULTITHREAD_OPENMP_H

#include <omp.h>

namespace cura
{

extern bool abort_execution;

#ifdef _OPENMP

class omp_lock_type
{
public:
    omp_lock_type()
    {
        omp_init_lock(&lock_object);
    }
    ~omp_lock_type()
    {
        omp_destroy_lock(&lock_object);
    }
    void lock()
    {
        omp_set_lock(&lock_object);
    }
    void unlock()
    {
        omp_unset_lock(&lock_object);
    }
    int test_lock()
    {
        return omp_test_lock(&lock_object);
    }
private:
    omp_lock_t lock_object;
    omp_lock_type( const omp_lock_type& ) = delete;
    omp_lock_type& operator=( const omp_lock_type& ) = delete;
};

class omp_nest_lock_type
{
public:
    omp_nest_lock_type()
    {
        omp_init_nest_lock(&lock_object);
    }
    ~omp_nest_lock_type()
    {
        omp_destroy_nest_lock(&lock_object);
    }
    void lock()
    {
        omp_set_nest_lock(&lock_object);
    }
    void unlock()
    {
        omp_unset_nest_lock(&lock_object);
    }
    int test_lock()
    {
        return omp_test_nest_lock(&lock_object);
    }
private:
    omp_nest_lock_t lock_object;
    omp_nest_lock_type( const omp_nest_lock_type& ) = delete;
    omp_nest_lock_type& operator=( const omp_nest_lock_type& ) = delete;
};

template <typename LockType>
class omp_try_lock_guard_t
{
public:
    omp_try_lock_guard_t( LockType& lock_)
    : lock(lock_)
    {
        has_lock = lock.test_lock();
    }
    ~omp_try_lock_guard_t()
    {
        if (has_lock)
        {
            lock.unlock();
        }
    }
    int isLocked()
    {
        return has_lock;
    }
private:
    LockType& lock;
    int has_lock;
    omp_try_lock_guard_t( const omp_try_lock_guard_t& ) = delete;
    omp_try_lock_guard_t<LockType>& operator=( const omp_try_lock_guard_t& ) = delete;
};

template <typename LockType>
class omp_lock_guard_t
{
public:
    omp_lock_guard_t( LockType& lock_)
    : lock(lock_)
    {
        lock.lock();
    }
    ~omp_lock_guard_t()
    {
        lock.unlock();
    }
private:
    LockType& lock;
    omp_lock_guard_t( const omp_lock_guard_t& ) = delete;
    omp_lock_guard_t& operator=( const omp_lock_guard_t& ) = delete;
};
#endif

inline bool checkMultithreadAbort()
{
    bool tmp_abort_execution;
#pragma omp atomic read
    tmp_abort_execution = abort_execution;
    return tmp_abort_execution;
}

inline void setMultithreadAbort()
{
#pragma omp atomic write
    abort_execution = true;
}

#ifdef _OPENMP
void handleMultithreadAbort();
#else
inline void handleMultithreadAbort(){}
#endif

#ifdef _OPENMP
#define MULTITHREAD_FOR_CATCH_EXCEPTION(code) \
    if (checkMultithreadAbort()) \
    { \
        continue; \
    } \
    try \
    { \
        code \
    } \
    catch (...) \
    { \
        setMultithreadAbort(); \
    }
#else
#define MULTITHREAD_FOR_CATCH_EXCEPTION(code) code
#endif

#ifdef _OPENMP
#define MULTITHREAD_TASK_CATCH_EXCEPTION(code) \
    if (!checkMultithreadAbort()) \
    { \
        try \
        { \
            code \
        } \
        catch (...) \
        { \
            setMultithreadAbort(); \
        } \
    }
#else
#define MULTITHREAD_TASK_CATCH_EXCEPTION(code) code
#endif

}//namespace cura

#endif // MULTITHREAD_OPENMP_H
