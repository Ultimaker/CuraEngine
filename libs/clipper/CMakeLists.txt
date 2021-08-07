message(STATUS "polyclipping: Using Shipped library.")

set(polyclipping_INCLUDE_DIRS "${CMAKE_SOURCE_DIR}/libs/clipper")
add_library(polyclipping STATIC ${polyclipping_INCLUDE_DIRS}/clipper.cpp)
target_include_directories(polyclipping PUBLIC ${polyclipping_INCLUDE_DIRS})
set_project_standards(polyclipping)
use_threads(polyclipping)

# Create an allias target for uniform access in main CMakeList.txt
add_library(polyclipping::polyclipping ALIAS polyclipping)