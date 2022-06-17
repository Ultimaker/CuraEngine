//Copyright (c) 2020 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef UTILS_MACROS_H
#define UTILS_MACROS_H

#include <atomic>

// macro to suppress unused parameter warnings from the compiler
#define UNUSED_PARAM(param) (void)(param)

// simple function to be used to reduce the number of times an error is logged
// no guarantees in a mutli-threaded context
#define RUN_ONCE(runcode) \
{ \
    static std::atomic<bool> code_ran = false; \
    if(!code_ran){ \
        bool expected = false; \
        if(code_ran.compare_exchange_strong(expected, true)) \
        { runcode; }\
    } \
}


#endif // UTILS_MACROS_H
