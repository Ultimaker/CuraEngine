##  Finds the Stb utility library on your computer.
#
#   If Stb is not found on your computer, this script also gives the option to
#   download the library and build it from source.
#
#   This script exports the following parameters for use if you find the Stb
#   package:
#   - stb_FOUND: Whether Stb has been found on your computer (or built from
#     source).
#   - stb_INCLUDE_DIRS: The directory where the header files of Stb are located.

#First try to find a PackageConfig for this library.
find_package(PkgConfig QUIET)
pkg_check_modules(PC_stb QUIET stb)

find_path(stb_INCLUDE_DIRS stb/stb_image_resize.h #Search for something that is a little less prone to false positives than just stb.h.
    HINTS ${PC_stb_INCLUDEDIR} ${PC_stb_INCLUDE_DIRS}
    PATHS "$ENV{PROGRAMFILES}" "$ENV{PROGRAMW6432}" "/usr/include"
    PATH_SUFFIXES include/stb stb include
)

include(FindPackageHandleStandardArgs)
set(_stb_find_required ${stb_FIND_REQUIRED}) #Temporarily set to optional so that we don't get a message when it's not found but you want to build from source.
set(_stb_find_quietly ${stb_FIND_QUIETLY})
set(stb_FIND_REQUIRED FALSE)
set(stb_FIND_QUIETLY TRUE)
find_package_handle_standard_args(stb DEFAULT_MSG stb_INCLUDE_DIRS)
set(stb_FIND_REQUIRED ${_stb_find_required})
set(stb_FIND_QUIETLY ${_stb_find_quietly})

set(CuraEngine_Download_Stb OFF)
if(stb_FOUND) #Found an existing installation.
    if(NOT stb_FIND_QUIETLY)
        message(STATUS "Found Stb installation at: ${stb_INCLUDE_DIRS}")
    endif()
else()
    #Then optionally clone Stb ourselves.
    option(BUILD_Stb "Build Stb from source." ON) #This is a lie actually, since Stb is header-only and doesn't need any building. We don't build the docs or tests.
    if(BUILD_Stb)
        if(NOT stb_FIND_QUIETLY)
            message(STATUS "Building Stb from source.")
        endif()

        include(ExternalProject)
        # Stb's commits in early February seems to cause the engine to fail compilation on Mac.
        ExternalProject_Add(stb
            GIT_REPOSITORY "https://github.com/nothings/stb.git"
            GIT_TAG d5d052c806eee2ca1f858cb58b2f062d9fa25b90
            UPDATE_DISCONNECTED TRUE
            CONFIGURE_COMMAND "" #We don't want to actually go and build/test/generate it. Just need to download the headers.
            BUILD_COMMAND ""
            INSTALL_COMMAND "" #Assume that the user doesn't want to install all dependencies on his system. We just need to get them for building the application.
        )
        set(CuraEngine_Download_Stb TRUE)
        set(stb_INCLUDE_DIRS "${CMAKE_CURRENT_BINARY_DIR}/stb-prefix/src/stb")
        set(stb_FOUND TRUE)
        if(NOT stb_FIND_QUIETLY)
            message(STATUS "Created Stb installation at: ${stb_INCLUDE_DIRS}")
        endif()
    elseif(NOT stb_FIND_QUIETLY) #Don't have an installation but don't want us to build it either? Screw you, then.
        if(stb_FIND_REQUIRED)
            message(FATAL_ERROR "Could NOT find Stb.")
        else()
            message(WARNING "Could NOT find Stb.")
        endif()
    endif()
endif()

if(stb_FOUND)
    add_library(stb::stb INTERFACE IMPORTED)
    set_target_properties(stb::stb PROPERTIES INTERFACE_INCLUDE_DIRECTORIES
            "${stb_INCLUDE_DIRS}")
endif()

mark_as_advanced(stb_INCLUDE_DIRS)
