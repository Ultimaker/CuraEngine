//Copyright (c) 2020 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef UTILS_MACROS_H
#define UTILS_MACROS_H

// macro to suppress unused parameter warnings from the compiler
#define UNUSED_PARAM(param) (void)(param)

// simple function to be used to reduce the number of times an error is logged
// no guarantees in a mutli-threaded context
#define RUN_ONCE(runcode) \
{ \
    static bool code_ran = 0; \
    if(!code_ran){ \
        code_ran = 1; \
        runcode; \
    } \
}


#endif // UTILS_MACROS_H
