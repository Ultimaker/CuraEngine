#
# - Try to find the polyclipping library
# this will define
#
#  polyclipping_FOUND - polyclipping was found
#  polyclipping_INCLUDE_DIRS - the polyclipping include directory
#  polyclipping_LIBRARIES - The libraries needed to use polyclipping
#  polyclipping_VERSION - The polyclipping library version
#  target polyclipping::polyclipping

#=============================================================================
# Copyright (c) 2017 Christophe Giboudeaux <christophe@krop.fr>
#
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. The name of the author may not be used to endorse or promote products
#    derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
# IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
# OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
# IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
# NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
# THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#=============================================================================


find_package(PkgConfig QUIET)
pkg_check_modules(PC_polyclipping QUIET polyclipping)

find_path(polyclipping_INCLUDE_DIRS
  NAMES clipper.hpp
  HINTS ${PC_polyclipping_INCLUDE_DIRS}
)

find_library(polyclipping_LIBRARIES
  NAMES polyclipping
  HINTS ${PC_polyclipping_LIBRARY_DIRS}
)

if(EXISTS ${polyclipping_INCLUDE_DIRS}/clipper.hpp)
  file(READ ${polyclipping_INCLUDE_DIRS}/clipper.hpp CLIPPER_H_CONTENT)
  string(REGEX MATCH "#define CLIPPER_VERSION[ ]+\"[0-9]+.[0-9]+.[0-9]+\"" CLIPPER_H_VERSION_MATCH ${CLIPPER_H_CONTENT})
  string(REGEX REPLACE "^.*CLIPPER_VERSION[ ]+\"([0-9]+.[0-9]+.[0-9]+).*$" "\\1" polyclipping_VERSION "${CLIPPER_H_VERSION_MATCH}")
endif()

include(FindPackageHandleStandardArgs)

find_package_handle_standard_args(polyclipping
    FOUND_VAR polyclipping_FOUND
    REQUIRED_VARS polyclipping_LIBRARIES polyclipping_INCLUDE_DIRS
    VERSION_VAR polyclipping_VERSION
)

if(polyclipping_FOUND)
  add_library(polyclipping::polyclipping INTERFACE IMPORTED)
  set_target_properties(polyclipping::polyclipping PROPERTIES INTERFACE_INCLUDE_DIRECTORIES
          "${polyclipping_INCLUDE_DIRS}")
  set_property(TARGET polyclipping::polyclipping PROPERTY INTERFACE_LINK_LIBRARIES
          "${polyclipping_LIBRARIES}")
endif()

mark_as_advanced(polyclipping_LIBRARIES polyclipping_INCLUDE_DIRS polyclipping_VERSION)