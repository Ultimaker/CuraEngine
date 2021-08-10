include(GNUInstallDirs) # Standard install dirs
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_FULL_LIBDIR}") # TODO: Check if this is still needed

# Uniform how we define BUILD_STATIC or BUILD_SHARED_LIBS (common practice)
if(DEFINED BUILD_STATIC)
    if(DEFINED BUILD_SHARED_LIBS)
        if(${BUILD_SHARED_LIBS} AND ${BUILD_STATIC})
            message(FATAL_ERROR "Conflicting arguments for BUILD_SHARED_LIBS=${BUILD_SHARED_LIBS} and BUILD_STATIC=${BUILD_STATIC}")
        endif()
    else()
        set(BUILD_SHARED_LIBS NOT ${BUILD_STATIC})
    endif()
else()
    if(NOT DEFINED BUILD_SHARED_LIBS)
        set(BUILD_SHARED_LIBS ON)
    endif()
endif()
message(STATUS "Setting BUILD_SHARED_LIBS to ${BUILD_SHARED_LIBS}")

# Set a default build type if none was specified
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message(STATUS "Setting build type to 'Release' as none was specified.")
    set(CMAKE_BUILD_TYPE
            Release
            CACHE STRING "Choose the type of build." FORCE)
    # Set the possible values of build type for cmake-gui, ccmake
    set_property(
            CACHE CMAKE_BUILD_TYPE
            PROPERTY STRINGS
            "Debug"
            "Release"
            "MinSizeRel"
            "RelWithDebInfo")
endif()

# Generate compile_commands.json to make it easier to work with clang based tools
message(STATUS "Generating compile commands to ${CMAKE_CURRENT_BINARY_DIR}/compile_commands.json")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(ENABLE_IPO "Enable Interprocedural Optimization, aka Link Time Optimization (LTO)" ON)
if(ENABLE_IPO)
    include(CheckIPOSupported)
    check_ipo_supported(
            RESULT
            result
            OUTPUT
            output)
    if(result)
        set(CMAKE_INTERPROCEDURAL_OPTIMIZATION TRUE)
    else()
        message(SEND_ERROR "IPO is not supported: ${output}")
    endif()
endif()

if (NOT MSVC)
    # Compile with the -fPIC options if supported
    if(DEFINED POSITION_INDEPENDENT_CODE)  # Use the user/Conan set value
        message(STATUS "Using POSITION_INDEPENDENT_CODE: ${POSITION_INDEPENDENT_CODE}")
    else()
        set(POSITION_INDEPENDENT_CODE ON) # Defaults to on
        message(STATUS "Setting POSITION_INDEPENDENT_CODE: ${POSITION_INDEPENDENT_CODE}")
    endif()
else()
    # Set Visual Studio flags MD/MDd or MT/MTd
    if(NOT DEFINED CMAKE_MSVC_RUNTIME_LIBRARY)
        if(BUILD_STATIC OR NOT BUILD_SHARED_LIBS)
            message(STATUS "Setting MT/MTd flags")
            set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
        else()
            message(STATUS "Setting MD/MDd flags")
            set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL")
        endif()
    endif()
endif()

# Use C++17 Standard
message(STATUS "Setting C++17 support with extensions off and standard required")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set common project options for the target
function(set_project_standards project_name)
    if(CMAKE_CXX_COMPILER_ID MATCHES ".*Clang")
        option(ENABLE_BUILD_WITH_TIME_TRACE "Enable -ftime-trace to generate time tracing .json files on clang" OFF)
        if(ENABLE_BUILD_WITH_TIME_TRACE)
            message(STATUS "Enabling time tracing for ${project_name}")
            add_compile_definitions(${project_name} PRIVATE -ftime-trace)
        endif()
        if (APPLE)
            message(STATUS "Compiling ${project_name} against libc++")
            target_compile_options(${project_name} PRIVATE "-stdlib=libc++")
        endif()
    endif()
endfunction()

# Ultimaker uniform Python linking method
function(use_python project_name)
    set(COMPONENTS ${ARGN})
    if(NOT DEFINED Python_VERSION)
        set(Python_VERSION
                3.8
                CACHE STRING "Python Version" FORCE)
        message(STATUS "Setting Python version to ${Python_VERSION}. Set Python_VERSION if you want to compile against an other version.")
    endif()
    if(APPLE)
        set(Python3_FIND_FRAMEWORK NEVER)
    endif()
    find_package(Python3 ${Python_VERSION} EXACT REQUIRED COMPONENTS ${COMPONENTS})
    target_link_libraries(${project_name} PRIVATE ${Python3_LIBRARIES})
    target_include_directories(${project_name} PUBLIC ${Python3_INCLUDE_DIRS})
    message(STATUS "Linking and building ${project_name} against Python ${Python3_VERSION}")
endfunction()

# Ultimaker uniform Thread linking method
function(use_threads project_name)
    message(STATUS "Enabling threading support for ${project_name}")
    set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
    set(THREADS_PREFER_PTHREAD_FLAG TRUE)
    find_package(Threads)
    target_link_libraries(${project_name} PRIVATE Threads::Threads)
endfunction()

# https://github.com/lefticus/cppbestpractices/blob/master/02-Use_the_Tools_Available.md
function(set_project_warnings project_name)
    message(STATUS "Setting warnings for ${project_name}")
    set(MSVC_WARNINGS
            /W4 # Baseline reasonable warnings
            /w14242 # 'identifier': conversion from 'type1' to 'type1', possible loss of data
            /w14254 # 'operator': conversion from 'type1:field_bits' to 'type2:field_bits', possible loss of data
            /w14263 # 'function': member function does not override any base class virtual member function
            /w14265 # 'classname': class has virtual functions, but destructor is not virtual instances of this class may not
            # be destructed correctly
            /w14287 # 'operator': unsigned/negative constant mismatch
            /we4289 # nonstandard extension used: 'variable': loop control variable declared in the for-loop is used outside
            # the for-loop scope
            /w14296 # 'operator': expression is always 'boolean_value'
            /w14311 # 'variable': pointer truncation from 'type1' to 'type2'
            /w14545 # expression before comma evaluates to a function which is missing an argument list
            /w14546 # function call before comma missing argument list
            /w14547 # 'operator': operator before comma has no effect; expected operator with side-effect
            /w14549 # 'operator': operator before comma has no effect; did you intend 'operator'?
            /w14555 # expression has no effect; expected expression with side- effect
            /w14619 # pragma warning: there is no warning number 'number'
            /w14640 # Enable warning on thread un-safe static member initialization
            /w14826 # Conversion from 'type1' to 'type_2' is sign-extended. This may cause unexpected runtime behavior.
            /w14905 # wide string literal cast to 'LPSTR'
            /w14906 # string literal cast to 'LPWSTR'
            /w14928 # illegal copy-initialization; more than one user-defined conversion has been implicitly applied
            /permissive- # standards conformance mode for MSVC compiler.
            )

    set(CLANG_WARNINGS
            -Wall
            -Wextra # reasonable and standard
            -Wshadow # warn the user if a variable declaration shadows one from a parent context
            -Wnon-virtual-dtor # warn the user if a class with virtual functions has a non-virtual destructor. This helps
            # catch hard to track down memory errors
            -Wold-style-cast # warn for c-style casts
            -Wcast-align # warn for potential performance problem casts
            -Wunused # warn on anything being unused
            -Woverloaded-virtual # warn if you overload (not override) a virtual function
            -Wpedantic # warn if non-standard C++ is used
            -Wconversion # warn on type conversions that may lose data
            -Wsign-conversion # warn on sign conversions
            -Wnull-dereference # warn if a null dereference is detected
            -Wdouble-promotion # warn if float is implicit promoted to double
            -Wformat=2 # warn on security issues around functions that format output (ie printf)
            )

    set(GCC_WARNINGS
            ${CLANG_WARNINGS}
            -Wmisleading-indentation # warn if indentation implies blocks where blocks do not exist
            -Wduplicated-cond # warn if if / else chain has duplicated conditions
            -Wduplicated-branches # warn if if / else branches have duplicated code
            -Wlogical-op # warn about logical operations being used where bitwise were probably wanted
            -Wuseless-cast # warn if you perform a cast to the same type
            )

    if(MSVC)
        set(PROJECT_WARNINGS ${MSVC_WARNINGS})
    elseif(CMAKE_CXX_COMPILER_ID MATCHES ".*Clang")
        set(PROJECT_WARNINGS ${CLANG_WARNINGS})
    elseif(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
        set(PROJECT_WARNINGS ${GCC_WARNINGS})
    else()
        message(AUTHOR_WARNING "No compiler warnings set for '${CMAKE_CXX_COMPILER_ID}' compiler.")
    endif()

    target_compile_options(${project_name} PRIVATE ${PROJECT_WARNINGS})

endfunction()

# This function will prevent in-source builds
function(AssureOutOfSourceBuilds)
    # make sure the user doesn't play dirty with symlinks
    get_filename_component(srcdir "${CMAKE_SOURCE_DIR}" REALPATH)
    get_filename_component(bindir "${CMAKE_BINARY_DIR}" REALPATH)

    # disallow in-source builds
    if("${srcdir}" STREQUAL "${bindir}")
        message("######################################################")
        message("Warning: in-source builds are disabled")
        message("Please create a separate build directory and run cmake from there")
        message("######################################################")
        message(FATAL_ERROR "Quitting configuration")
    endif()
endfunction()

option(ALLOW_IN_SOURCE_BUILD "Allow building in your source folder. Strongly discouraged" OFF)
if(NOT ALLOW_IN_SOURCE_BUILD)
    assureoutofsourcebuilds()
endif()

function(enable_sanitizers project_name)

    if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID MATCHES ".*Clang")
        option(ENABLE_COVERAGE "Enable coverage reporting for gcc/clang" FALSE)

        if(ENABLE_COVERAGE)
            target_compile_options(${project_name} INTERFACE --coverage -O0 -g)
            target_link_libraries(${project_name} INTERFACE --coverage)
        endif()

        set(SANITIZERS "")

        option(ENABLE_SANITIZER_ADDRESS "Enable address sanitizer" FALSE)
        if(ENABLE_SANITIZER_ADDRESS)
            list(APPEND SANITIZERS "address")
        endif()

        option(ENABLE_SANITIZER_LEAK "Enable leak sanitizer" FALSE)
        if(ENABLE_SANITIZER_LEAK)
            list(APPEND SANITIZERS "leak")
        endif()

        option(ENABLE_SANITIZER_UNDEFINED_BEHAVIOR "Enable undefined behavior sanitizer" FALSE)
        if(ENABLE_SANITIZER_UNDEFINED_BEHAVIOR)
            list(APPEND SANITIZERS "undefined")
        endif()

        option(ENABLE_SANITIZER_THREAD "Enable thread sanitizer" FALSE)
        if(ENABLE_SANITIZER_THREAD)
            if("address" IN_LIST SANITIZERS OR "leak" IN_LIST SANITIZERS)
                message(WARNING "Thread sanitizer does not work with Address and Leak sanitizer enabled")
            else()
                list(APPEND SANITIZERS "thread")
            endif()
        endif()

        option(ENABLE_SANITIZER_MEMORY "Enable memory sanitizer" FALSE)
        if(ENABLE_SANITIZER_MEMORY AND CMAKE_CXX_COMPILER_ID MATCHES ".*Clang")
            if("address" IN_LIST SANITIZERS
                    OR "thread" IN_LIST SANITIZERS
                    OR "leak" IN_LIST SANITIZERS)
                message(WARNING "Memory sanitizer does not work with Address, Thread and Leak sanitizer enabled")
            else()
                list(APPEND SANITIZERS "memory")
            endif()
        endif()

        list(
                JOIN
                SANITIZERS
                ","
                LIST_OF_SANITIZERS)

    endif()

    if(LIST_OF_SANITIZERS)
        if(NOT
                "${LIST_OF_SANITIZERS}"
                STREQUAL
                "")
            target_compile_options(${project_name} INTERFACE -fsanitize=${LIST_OF_SANITIZERS})
            target_link_options(${project_name} INTERFACE -fsanitize=${LIST_OF_SANITIZERS})
        endif()
    endif()

endfunction()

option(ENABLE_CPPCHECK "Enable static analysis with cppcheck" OFF)
option(ENABLE_CLANG_TIDY "Enable static analysis with clang-tidy" OFF)
option(ENABLE_INCLUDE_WHAT_YOU_USE "Enable static analysis with include-what-you-use" OFF)

if(ENABLE_CPPCHECK)
    find_program(CPPCHECK cppcheck)
    if(CPPCHECK)
        message(STATUS "Using cppcheck")
        set(CMAKE_CXX_CPPCHECK
                ${CPPCHECK}
                --suppress=missingInclude
                --enable=all
                --inline-suppr
                --inconclusive
                -i
                ${CMAKE_SOURCE_DIR}/imgui/lib)
    else()
        message(WARNING "cppcheck requested but executable not found")
    endif()
endif()

if(ENABLE_CLANG_TIDY)
    find_program(CLANGTIDY clang-tidy)
    if(CLANGTIDY)
        message(STATUS "Using clang-tidy")
        set(CMAKE_CXX_CLANG_TIDY ${CLANGTIDY} -extra-arg=-Wno-unknown-warning-option)
    else()
        message(WARNING "clang-tidy requested but executable not found")
    endif()
endif()

if(ENABLE_INCLUDE_WHAT_YOU_USE)
    find_program(INCLUDE_WHAT_YOU_USE include-what-you-use)
    if(INCLUDE_WHAT_YOU_USE)
        message(STATUS "Using include-what-you-use")
        set(CMAKE_CXX_INCLUDE_WHAT_YOU_USE ${INCLUDE_WHAT_YOU_USE})
    else()
        message(WARNING "include-what-you-use requested but executable not found")
    endif()
endif()