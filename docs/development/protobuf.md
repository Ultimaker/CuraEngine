


# protobuf/3.17.1

---



<br>

## protobuf/3.17.1 dependencies

* [zlib/1.2.12](https://conan.io/center/zlib)

<br>

## Using the protobuf Conan Package

<br>

Conan integrates with different build systems. You can declare which build system you want your project to use setting in the **[generators]** section of the [conanfile.txt](https://docs.conan.io/en/latest/reference/conanfile_txt.html#generators) or using the **generators** attribute in the [conanfile.py](https://docs.conan.io/en/latest/reference/conanfile/attributes.html#generators). Here, there is some basic information you can use to integrate **protobuf** in your own project. For more detailed information, please [check the Conan documentation](https://docs.conan.io/en/latest/getting_started.html).



<br>

## Using protobuf with CMake

<br>

### [Conan CMake generators](https://docs.conan.io/en/latest/reference/conanfile/tools/cmake.html)

<br>

* [CMakeDeps](https://docs.conan.io/en/latest/reference/conanfile/tools/cmake/cmakedeps.html): generates information about where the **protobuf** library and its dependencies  ( [zlib](https://conan.io/center/zlib)) are installed together with other information like version, flags, and directory data or configuration. CMake will use this files when you invoke ``find_package()`` in your *CMakeLists.txt*.

* [CMakeToolchain](https://docs.conan.io/en/latest/reference/conanfile/tools/cmake/cmaketoolchain.html): generates a CMake toolchain file the you can later invoke with CMake in the command line using `-DCMAKE_TOOLCHAIN_FILE=conantoolchain.cmake`.

Declare these generators in your **conanfile.txt** along with your **protobuf** dependency like:

```ini
[requires]
protobuf/3.17.1

[generators]
CMakeDeps
CMakeToolchain
```

<br>

To use **protobuf** in a simple CMake project with this structure:

```shell
.
|-- CMakeLists.txt
|-- conanfile.txt
`-- src
    `-- main..cpp
```

<br>

Your **CMakeLists.txt** could look similar to this, using the global **protobuf::protobuf** CMake's target:

```cmake
cmake_minimum_required(VERSION 3.15)
project(protobuf_project CXX)

find_package(protobuf)

add_executable(${PROJECT_NAME} src/main.cpp)

# Use the global target
target_link_libraries(${PROJECT_NAME} protobuf::protobuf)
```

<br>

To install **protobuf/3.17.1**, its dependencies and build your project, you just have to do:

```shell
# for Linux/macOS
$ conan install . --install-folder cmake-build-release --build=missing
$ cmake . -DCMAKE_TOOLCHAIN_FILE=cmake-build-release/conan_toolchain.cmake
$ cmake --build .

# for Windows and Visual Studio 2017
$ conan install . --output-folder cmake-build --build=missing
$ cmake . -G "Visual Studio 15 2017" -DCMAKE_TOOLCHAIN_FILE=cmake-build/conan_toolchain.cmake
$ cmake --build . --config Release
```



<br>



As the protobuf Conan package defines components you can link only that desired part of the library in your project. For example, linking only with the protobuf **libprotobuf** component, through the **protobuf::libprotobuf** target.

```cmake
...
# Link just to protobuf libprotobuf component
target_link_libraries(${PROJECT_NAME} protobuf::libprotobuf)
```

<br>

To check all the available components for **protobuf** Conan package, please check the dedicated section at the end of this document.




<br>

### Declared CMake build modules

<br>

#### lib/cmake/protobuf/protobuf-generate.cmake
  ```cmake
  # User options
  include("${CMAKE_CURRENT_LIST_DIR}/protobuf-options.cmake")

  # Depend packages
  # BEGIN CONAN PATCH
  #_protobuf_FIND_ZLIB@
  # END CONAN PATCH

  # Imported targets
  if(NOT TARGET protobuf::protoc)
      if(CMAKE_CROSSCOMPILING)
          find_program(PROTOC_PROGRAM protoc PATHS ENV PATH NO_DEFAULT_PATH)
      endif()
      if(NOT PROTOC_PROGRAM)
          set(PROTOC_PROGRAM "${CMAKE_CURRENT_LIST_DIR}/../../../bin/protoc")
      endif()
      get_filename_component(PROTOC_PROGRAM "${PROTOC_PROGRAM}" ABSOLUTE)
      set(Protobuf_PROTOC_EXECUTABLE ${PROTOC_PROGRAM} CACHE FILEPATH "The protoc compiler")
      add_executable(protobuf::protoc IMPORTED)
      set_property(TARGET protobuf::protoc PROPERTY IMPORTED_LOCATION ${Protobuf_PROTOC_EXECUTABLE})
  endif()


  function(protobuf_generate)
    include(CMakeParseArguments)

    set(_options APPEND_PATH)
    set(_singleargs LANGUAGE OUT_VAR EXPORT_MACRO PROTOC_OUT_DIR PLUGIN)
    if(COMMAND target_sources)
      list(APPEND _singleargs TARGET)
    endif()
    set(_multiargs PROTOS IMPORT_DIRS GENERATE_EXTENSIONS PROTOC_OPTIONS)

    cmake_parse_arguments(protobuf_generate "${_options}" "${_singleargs}" "${_multiargs}" "${ARGN}")

    if(NOT protobuf_generate_PROTOS AND NOT protobuf_generate_TARGET)
      message(SEND_ERROR "Error: protobuf_generate called without any targets or source files")
      return()
    endif()

    if(NOT protobuf_generate_OUT_VAR AND NOT protobuf_generate_TARGET)
      message(SEND_ERROR "Error: protobuf_generate called without a target or output variable")
      return()
    endif()

    if(NOT protobuf_generate_LANGUAGE)
      set(protobuf_generate_LANGUAGE cpp)
    endif()
    string(TOLOWER ${protobuf_generate_LANGUAGE} protobuf_generate_LANGUAGE)

    if(NOT protobuf_generate_PROTOC_OUT_DIR)
      set(protobuf_generate_PROTOC_OUT_DIR ${CMAKE_CURRENT_BINARY_DIR})
    endif()

    if(protobuf_generate_EXPORT_MACRO AND protobuf_generate_LANGUAGE STREQUAL cpp)
      set(_dll_export_decl "dllexport_decl=${protobuf_generate_EXPORT_MACRO}:")
    endif()
    
    if(protobuf_generate_PLUGIN)
        set(_plugin "--plugin=${protobuf_generate_PLUGIN}")
    endif()

    if(NOT protobuf_generate_GENERATE_EXTENSIONS)
      if(protobuf_generate_LANGUAGE STREQUAL cpp)
        set(protobuf_generate_GENERATE_EXTENSIONS .pb.h .pb.cc)
      elseif(protobuf_generate_LANGUAGE STREQUAL python)
        set(protobuf_generate_GENERATE_EXTENSIONS _pb2.py)
      else()
        message(SEND_ERROR "Error: protobuf_generate given unknown Language ${LANGUAGE}, please provide a value for GENERATE_EXTENSIONS")
        return()
      endif()
    endif()

    if(protobuf_generate_TARGET)
      get_target_property(_source_list ${protobuf_generate_TARGET} SOURCES)
      foreach(_file ${_source_list})
        if(_file MATCHES "proto$")
          list(APPEND protobuf_generate_PROTOS ${_file})
        endif()
      endforeach()
    endif()

    if(NOT protobuf_generate_PROTOS)
      message(SEND_ERROR "Error: protobuf_generate could not find any .proto files")
      return()
    endif()

    if(protobuf_generate_APPEND_PATH)
      # Create an include path for each file specified
      foreach(_file ${protobuf_generate_PROTOS})
        get_filename_component(_abs_file ${_file} ABSOLUTE)
        get_filename_component(_abs_path ${_abs_file} PATH)
        list(FIND _protobuf_include_path ${_abs_path} _contains_already)
        if(${_contains_already} EQUAL -1)
            list(APPEND _protobuf_include_path -I ${_abs_path})
        endif()
      endforeach()
    endif()

    foreach(DIR ${protobuf_generate_IMPORT_DIRS})
      get_filename_component(ABS_PATH ${DIR} ABSOLUTE)
      list(FIND _protobuf_include_path ${ABS_PATH} _contains_already)
      if(${_contains_already} EQUAL -1)
          list(APPEND _protobuf_include_path -I ${ABS_PATH})
      endif()
    endforeach()

    if(NOT _protobuf_include_path)
      set(_protobuf_include_path -I ${CMAKE_CURRENT_SOURCE_DIR})
    endif()

    set(_generated_srcs_all)
    foreach(_proto ${protobuf_generate_PROTOS})
      get_filename_component(_abs_file ${_proto} ABSOLUTE)
      get_filename_component(_abs_dir ${_abs_file} DIRECTORY)

      get_filename_component(_file_full_name ${_proto} NAME)
      string(FIND "${_file_full_name}" "." _file_last_ext_pos REVERSE)
      string(SUBSTRING "${_file_full_name}" 0 ${_file_last_ext_pos} _basename)

      set(_suitable_include_found FALSE)
      foreach(DIR ${_protobuf_include_path})
        if(NOT DIR STREQUAL "-I")
          file(RELATIVE_PATH _rel_dir ${DIR} ${_abs_dir})
          string(FIND "${_rel_dir}" "../" _is_in_parent_folder)
          if (NOT ${_is_in_parent_folder} EQUAL 0)
            set(_suitable_include_found TRUE)
            break()
          endif()
        endif()
      endforeach()

      if(NOT _suitable_include_found)
        message(SEND_ERROR "Error: protobuf_generate could not find any correct proto include directory.")
        return()
      endif()

      set(_generated_srcs)
      foreach(_ext ${protobuf_generate_GENERATE_EXTENSIONS})
        list(APPEND _generated_srcs "${protobuf_generate_PROTOC_OUT_DIR}/${_rel_dir}/${_basename}${_ext}")
      endforeach()
      list(APPEND _generated_srcs_all ${_generated_srcs})

      add_custom_command(
        OUTPUT ${_generated_srcs}
        COMMAND  protobuf::protoc
        ARGS ${protobuf_generate_PROTOC_OPTIONS} --${protobuf_generate_LANGUAGE}_out ${_dll_export_decl}${protobuf_generate_PROTOC_OUT_DIR} ${_plugin} ${_protobuf_include_path} ${_abs_file}
        DEPENDS ${_abs_file} protobuf::protoc
        COMMENT "Running ${protobuf_generate_LANGUAGE} protocol buffer compiler on ${_proto}. Custom options: ${protobuf_generate_PROTOC_OPTIONS}"
        VERBATIM )
    endforeach()

    set_source_files_properties(${_generated_srcs_all} PROPERTIES GENERATED TRUE)
    if(protobuf_generate_OUT_VAR)
      set(${protobuf_generate_OUT_VAR} ${_generated_srcs_all} PARENT_SCOPE)
    endif()
    if(protobuf_generate_TARGET)
      target_sources(${protobuf_generate_TARGET} PRIVATE ${_generated_srcs_all})
    endif()

  endfunction()

  # CMake FindProtobuf module compatible file
  if(protobuf_MODULE_COMPATIBLE)
    include("${CMAKE_CURRENT_LIST_DIR}/protobuf-module.cmake")
  endif()

  ```#### lib/cmake/protobuf/protobuf-module.cmake
  ```cmake
  # This file contains backwards compatibility patches for various legacy functions and variables
  # Functions

  function(PROTOBUF_GENERATE_CPP SRCS HDRS)
    cmake_parse_arguments(protobuf_generate_cpp "" "EXPORT_MACRO" "" ${ARGN})

    set(_proto_files "${protobuf_generate_cpp_UNPARSED_ARGUMENTS}")
    if(NOT _proto_files)
      message(SEND_ERROR "Error: PROTOBUF_GENERATE_CPP() called without any proto files")
      return()
    endif()

    if(PROTOBUF_GENERATE_CPP_APPEND_PATH)
      set(_append_arg APPEND_PATH)
    endif()

    if(DEFINED Protobuf_IMPORT_DIRS)
      set(_import_arg IMPORT_DIRS ${Protobuf_IMPORT_DIRS})
    endif()

    set(_outvar)
    protobuf_generate(${_append_arg} LANGUAGE cpp EXPORT_MACRO ${protobuf_generate_cpp_EXPORT_MACRO} OUT_VAR _outvar ${_import_arg} PROTOS ${_proto_files})

    set(${SRCS})
    set(${HDRS})
    foreach(_file ${_outvar})
      if(_file MATCHES "cc$")
        list(APPEND ${SRCS} ${_file})
      else()
        list(APPEND ${HDRS} ${_file})
      endif()
    endforeach()
    set(${SRCS} ${${SRCS}} PARENT_SCOPE)
    set(${HDRS} ${${HDRS}} PARENT_SCOPE)
  endfunction()

  function(PROTOBUF_GENERATE_PYTHON SRCS)
    if(NOT ARGN)
      message(SEND_ERROR "Error: PROTOBUF_GENERATE_PYTHON() called without any proto files")
      return()
    endif()

    if(PROTOBUF_GENERATE_CPP_APPEND_PATH)
      set(_append_arg APPEND_PATH)
    endif()

    if(DEFINED Protobuf_IMPORT_DIRS)
      set(_import_arg IMPORT_DIRS ${Protobuf_IMPORT_DIRS})
    endif()

    set(_outvar)
    protobuf_generate(${_append_arg} LANGUAGE python OUT_VAR _outvar ${_import_arg} PROTOS ${ARGN})
    set(${SRCS} ${_outvar} PARENT_SCOPE)
  endfunction()

  # Environment

  # Backwards compatibility
  # Define camel case versions of input variables
  foreach(UPPER
      PROTOBUF_SRC_ROOT_FOLDER
      PROTOBUF_IMPORT_DIRS
      PROTOBUF_DEBUG
      PROTOBUF_LIBRARY
      PROTOBUF_PROTOC_LIBRARY
      PROTOBUF_INCLUDE_DIR
      PROTOBUF_PROTOC_EXECUTABLE
      PROTOBUF_LIBRARY_DEBUG
      PROTOBUF_PROTOC_LIBRARY_DEBUG
      PROTOBUF_LITE_LIBRARY
      PROTOBUF_LITE_LIBRARY_DEBUG
      )
      if (DEFINED ${UPPER})
          string(REPLACE "PROTOBUF_" "Protobuf_" Camel ${UPPER})
          if (NOT DEFINED ${Camel})
              set(${Camel} ${${UPPER}})
          endif()
      endif()
  endforeach()

  if(0)
  if(DEFINED Protobuf_SRC_ROOT_FOLDER)
    message(AUTHOR_WARNING "Variable Protobuf_SRC_ROOT_FOLDER defined, but not"
      " used in CONFIG mode")
  endif()

  include(SelectLibraryConfigurations)

  # Internal function: search for normal library as well as a debug one
  #    if the debug one is specified also include debug/optimized keywords
  #    in *_LIBRARIES variable
  function(_protobuf_find_libraries name filename)
    if(${name}_LIBRARIES)
      # Use result recorded by a previous call.
    elseif(${name}_LIBRARY)
      # Honor cache entry used by CMake 3.5 and lower.
      set(${name}_LIBRARIES "${${name}_LIBRARY}" PARENT_SCOPE)
    else()
      get_target_property(${name}_LIBRARY_RELEASE protobuf::lib${filename}
        LOCATION_RELEASE)
      get_target_property(${name}_LIBRARY_RELWITHDEBINFO protobuf::lib${filename}
        LOCATION_RELWITHDEBINFO)
      get_target_property(${name}_LIBRARY_MINSIZEREL protobuf::lib${filename}
        LOCATION_MINSIZEREL)
      get_target_property(${name}_LIBRARY_DEBUG protobuf::lib${filename}
        LOCATION_DEBUG)

      select_library_configurations(${name})
      set(${name}_LIBRARY ${${name}_LIBRARY} PARENT_SCOPE)
      set(${name}_LIBRARIES ${${name}_LIBRARIES} PARENT_SCOPE)
    endif()
  endfunction()

  # Internal function: find threads library
  function(_protobuf_find_threads)
      set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
      find_package(Threads)
      if(Threads_FOUND)
          list(APPEND PROTOBUF_LIBRARIES ${CMAKE_THREAD_LIBS_INIT})
          set(PROTOBUF_LIBRARIES "${PROTOBUF_LIBRARIES}" PARENT_SCOPE)
      endif()
  endfunction()

  #
  # Main.
  #

  # By default have PROTOBUF_GENERATE_CPP macro pass -I to protoc
  # for each directory where a proto file is referenced.
  if(NOT DEFINED PROTOBUF_GENERATE_CPP_APPEND_PATH)
    set(PROTOBUF_GENERATE_CPP_APPEND_PATH TRUE)
  endif()

  # The Protobuf library
  _protobuf_find_libraries(Protobuf protobuf)

  # The Protobuf Lite library
  _protobuf_find_libraries(Protobuf_LITE protobuf-lite)

  # The Protobuf Protoc Library
  _protobuf_find_libraries(Protobuf_PROTOC protoc)

  if(UNIX)
    _protobuf_find_threads()
  endif()

  # Set the include directory
  get_target_property(Protobuf_INCLUDE_DIRS protobuf::libprotobuf
    INTERFACE_INCLUDE_DIRECTORIES)

  # Set the protoc Executable
  get_target_property(Protobuf_PROTOC_EXECUTABLE protobuf::protoc
    IMPORTED_LOCATION_RELEASE)
  if(NOT EXISTS "${Protobuf_PROTOC_EXECUTABLE}")
    get_target_property(Protobuf_PROTOC_EXECUTABLE protobuf::protoc
      IMPORTED_LOCATION_RELWITHDEBINFO)
  endif()
  if(NOT EXISTS "${Protobuf_PROTOC_EXECUTABLE}")
    get_target_property(Protobuf_PROTOC_EXECUTABLE protobuf::protoc
      IMPORTED_LOCATION_MINSIZEREL)
  endif()
  if(NOT EXISTS "${Protobuf_PROTOC_EXECUTABLE}")
    get_target_property(Protobuf_PROTOC_EXECUTABLE protobuf::protoc
      IMPORTED_LOCATION_DEBUG)
  endif()
  if(NOT EXISTS "${Protobuf_PROTOC_EXECUTABLE}")
    get_target_property(Protobuf_PROTOC_EXECUTABLE protobuf::protoc
      IMPORTED_LOCATION_NOCONFIG)
  endif()

  # Version info variable
  set(Protobuf_VERSION "3.17.1.0")

  include(FindPackageHandleStandardArgs)
  FIND_PACKAGE_HANDLE_STANDARD_ARGS(Protobuf
      REQUIRED_VARS Protobuf_PROTOC_EXECUTABLE Protobuf_LIBRARIES Protobuf_INCLUDE_DIRS
      VERSION_VAR Protobuf_VERSION
  )

  # Backwards compatibility
  endif()
  foreach(Camel
      Protobuf_VERSION
      Protobuf_SRC_ROOT_FOLDER
      Protobuf_IMPORT_DIRS
      Protobuf_DEBUG
      Protobuf_INCLUDE_DIRS
      Protobuf_LIBRARIES
      Protobuf_PROTOC_LIBRARIES
      Protobuf_LITE_LIBRARIES
      Protobuf_LIBRARY
      Protobuf_PROTOC_LIBRARY
      Protobuf_INCLUDE_DIR
      Protobuf_PROTOC_EXECUTABLE
      Protobuf_LIBRARY_DEBUG
      Protobuf_PROTOC_LIBRARY_DEBUG
      Protobuf_LITE_LIBRARY
      Protobuf_LITE_LIBRARY_DEBUG
      )
      string(TOUPPER ${Camel} UPPER)
      set(${UPPER} ${${Camel}})
  endforeach()

  ```#### lib/cmake/protobuf/protobuf-options.cmake
  ```cmake
  # Verbose output
  option(protobuf_VERBOSE "Enable for verbose output" OFF)
  mark_as_advanced(protobuf_VERBOSE)

  # FindProtobuf module compatible
  option(protobuf_MODULE_COMPATIBLE "CMake built-in FindProtobuf.cmake module compatible" OFF)
  mark_as_advanced(protobuf_MODULE_COMPATIBLE)

  ```



<br>

## Using protobuf with Visual Studio

<br>

### [Visual Studio Conan generators](https://docs.conan.io/en/latest/reference/conanfile/tools/microsoft.html)

<br>

* [MSBuildDeps](https://docs.conan.io/en/latest/reference/conanfile/tools/microsoft.html#msbuilddeps): generates the **conandeps.props** properties file with information about where the **protobuf** library and its dependencies  ( [zlib](https://conan.io/center/zlib)) are installed together with other information like version, flags, and directory data or configuration.

* [MSBuildToolchain](https://docs.conan.io/en/latest/reference/conanfile/tools/microsoft.html#msbuildtoolchain): Generates the **conantoolchain.props** properties file with the current package configuration, settings, and options.

Declare these generators in your **conanfile.txt** along with your **protobuf** dependency like:

```ini
[requires]
protobuf/3.17.1

[generators]
MSBuildDeps
MSBuildToolchain
```

<br>

Please, [check the Conan documentation](https://docs.conan.io/en/latest/reference/conanfile/tools/microsoft.html) for more detailed information on how to add these properties files to your Visual Studio projects.



<br>

## Using protobuf with Autotools and pkg-config

<br>

### [Autotools Conan generators](https://docs.conan.io/en/latest/reference/conanfile/tools/gnu.html)

<br>

* [AutotoolsToolchain](https://docs.conan.io/en/latest/reference/conanfile/tools/gnu/autotoolstoolchain.html): generates the **conanautotoolstoolchain.sh/bat** script translating information from the current package configuration, settings, and options setting some enviroment variables for Autotools like: ``CPPFLAGS``, ``CXXFLAGS``, ``CFLAGS`` and ``LDFLAGS``. It will also generate a ``deactivate_conanautotoolstoolchain.sh/bat`` so you can restore your environment.

* [AutotoolsDeps](https://docs.conan.io/en/latest/reference/conanfile/tools/gnu/autotoolsdeps.html): generates the **conanautotoolsdeps.sh/bat** script with information about where the **protobuf** library and its dependencies  ( [zlib](https://conan.io/center/zlib)) are installed together with other information like version, flags, and directory data or configuration. This is done setting some enviroment variables for Autotools like: ``LIBS``, ``CPPFLAGS``,``CXXFLAGS``, ``CFLAGS`` and ``LDFLAGS``.  It will also generate a ``deactivate_conanautotoolsdeps.sh/bat`` so you can restore your environment.

Declare these generators in your **conanfile.txt** along with your **protobuf** dependency like:

```ini
[requires]
protobuf/3.17.1

[generators]
AutotoolsToolchain
AutotoolsDeps
```

<br>

Then, building your project is as easy as:

```shell
$ conan install .
# set the environment variables for Autotools
$ source conanautotoolstoolchain.sh
$ source conanautotoolsdeps.sh
$ ./configure
$ make

# restore the environment after the build is completed
$ source deactivate_conanautotoolstoolchain.sh
$ source deactivate_conanautotoolsdeps.sh
```

<br>

### [pkg-config Conan generator](https://docs.conan.io/en/latest/reference/conanfile/tools/gnu/pkgconfigdeps.html)

<br>

* [PkgConfigDeps](https://docs.conan.io/en/latest/reference/conanfile/tools/gnu/pkgconfigdeps.html): generates the **protobuf_full_package.pc** file (and the ones corresponding to **protobuf** dependencies) with information about the dependencies that can be later used by the **pkg-config** tool pkg-config to collect data about the libraries Conan installed.

<br>

You can use this generator instead of the **AutotoolsDeps** one:

```ini
[requires]
protobuf/3.17.1

[generators]
AutotoolsToolchain
PkgConfigDeps
```

<br>

And then using **pkg-config** to set the environment variables you want, like:

```shell
$ conan install .
# set the environment variables for Autotools
$ source conanautotoolstoolchain.sh

# You will have to set the PKG_CONFIG_PATH to where Conan created the protobuf_full_package.pc file
$ export CPPFLAGS="$CPPFLAGS $(pkg-config --cflags protobuf)"
$ export LIBS="$LIBS $(pkg-config --libs-only-l protobuf)"
$ export LDFLAGS="$LDFLAGS $(pkg-config --libs-only-L --libs-only-other protobuf)"

$ ./configure
$ make

# restore the environment after the build is completed
$ source deactivate_conanautotoolstoolchain.sh
```



<br>

As the protobuf Conan package defines components you can use them to link only that desired part of the library in your project.  To check all the available components for **protobuf** Conan package, and the corresponding *.pc* files names, please check the dedicated section at the end of this document.


<br>

## Other build systems

<br>

Please, [check the Conan documentation](https://docs.conan.io/en/latest/reference/conanfile/tools.html) for other integrations besides the ones listed in this document.





<br>

## Declared components for protobuf

<br>

These are all the declared components for the **protobuf** Conan package:
* Component **libprotobuf**:
  * CMake target name: ``protobuf::libprotobuf``
  * pkg-config *.pc* file: **protobuf.pc**
  * Requires other components: **zlib::zlib**
  * Links to libraries: **protobuf**
  * Systems libs: **pthread**
* Component **libprotoc**:
  * CMake target name: ``protobuf::libprotoc``
  * pkg-config *.pc* file: **protobuf_full_package-libprotoc.pc**
  * Requires other components: **libprotobuf**
  * Links to libraries: **protoc**


<br>

## Exposed header files for protobuf

<br>

```cpp
#include <google/protobuf/map_entry_lite.h>
#include <google/protobuf/arenastring.h>
#include <google/protobuf/extension_set.h>
#include <google/protobuf/map_type_handler.h>
#include <google/protobuf/type.proto>
#include <google/protobuf/any.pb.h>
#include <google/protobuf/port.h>
#include <google/protobuf/type.pb.h>
#include <google/protobuf/field_mask.pb.h>
#include <google/protobuf/map_entry.h>
#include <google/protobuf/descriptor.proto>
#include <google/protobuf/arena_impl.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/parse_context.h>
#include <google/protobuf/generated_message_table_driven.h>
#include <google/protobuf/empty.pb.h>
#include <google/protobuf/repeated_field.h>
#include <google/protobuf/generated_enum_reflection.h>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/map.h>
#include <google/protobuf/message.h>
#include <google/protobuf/duration.pb.h>
#include <google/protobuf/metadata_lite.h>
#include <google/protobuf/wrappers.proto>
#include <google/protobuf/extension_set_inl.h>
#include <google/protobuf/unknown_field_set.h>
#include <google/protobuf/message_lite.h>
#include <google/protobuf/field_mask.proto>
#include <google/protobuf/descriptor.pb.h>
#include <google/protobuf/any.h>
#include <google/protobuf/duration.proto>
#include <google/protobuf/timestamp.pb.h>
#include <google/protobuf/wrappers.pb.h>
#include <google/protobuf/generated_enum_util.h>
#include <google/protobuf/map_field.h>
#include <google/protobuf/source_context.pb.h>
#include <google/protobuf/empty.proto>
#include <google/protobuf/struct.pb.h>
#include <google/protobuf/implicit_weak_message.h>
#include <google/protobuf/any.proto>
#include <google/protobuf/port_undef.inc>
#include <google/protobuf/reflection.h>
#include <google/protobuf/metadata.h>
#include <google/protobuf/dynamic_message.h>
#include <google/protobuf/text_format.h>
#include <google/protobuf/reflection_ops.h>
#include <google/protobuf/has_bits.h>
#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/api.proto>
#include <google/protobuf/api.pb.h>
#include <google/protobuf/arena.h>
#include <google/protobuf/wire_format.h>
#include <google/protobuf/struct.proto>
#include <google/protobuf/map_field_lite.h>
#include <google/protobuf/port_def.inc>
#include <google/protobuf/descriptor_database.h>
#include <google/protobuf/service.h>
#include <google/protobuf/wire_format_lite.h>
#include <google/protobuf/source_context.proto>
#include <google/protobuf/timestamp.proto>
#include <google/protobuf/map_field_inl.h>
#include <google/protobuf/io/printer.h>
#include <google/protobuf/io/strtod.h>
#include <google/protobuf/io/zero_copy_stream.h>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/io/zero_copy_stream_impl_lite.h>
#include <google/protobuf/io/gzip_stream.h>
#include <google/protobuf/io/io_win32.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/io/tokenizer.h>
#include <google/protobuf/stubs/once.h>
#include <google/protobuf/stubs/port.h>
#include <google/protobuf/stubs/mutex.h>
#include <google/protobuf/stubs/common.h>
#include <google/protobuf/stubs/macros.h>
#include <google/protobuf/stubs/status.h>
#include <google/protobuf/stubs/template_util.h>
#include <google/protobuf/stubs/bytestream.h>
#include <google/protobuf/stubs/map_util.h>
#include <google/protobuf/stubs/logging.h>
#include <google/protobuf/stubs/stl_util.h>
#include <google/protobuf/stubs/callback.h>
#include <google/protobuf/stubs/strutil.h>
#include <google/protobuf/stubs/hash.h>
#include <google/protobuf/stubs/casts.h>
#include <google/protobuf/stubs/stringpiece.h>
#include <google/protobuf/stubs/platform_macros.h>
#include <google/protobuf/util/message_differencer.h>
#include <google/protobuf/util/delimited_message_util.h>
#include <google/protobuf/util/field_mask_util.h>
#include <google/protobuf/util/field_comparator.h>
#include <google/protobuf/util/type_resolver.h>
#include <google/protobuf/util/type_resolver_util.h>
#include <google/protobuf/util/json_util.h>
#include <google/protobuf/util/time_util.h>
#include <google/protobuf/compiler/plugin.h>
#include <google/protobuf/compiler/plugin.pb.h>
#include <google/protobuf/compiler/parser.h>
#include <google/protobuf/compiler/command_line_interface.h>
#include <google/protobuf/compiler/code_generator.h>
#include <google/protobuf/compiler/plugin.proto>
#include <google/protobuf/compiler/importer.h>
#include <google/protobuf/compiler/objectivec/objectivec_helpers.h>
#include <google/protobuf/compiler/objectivec/objectivec_generator.h>
#include <google/protobuf/compiler/python/python_generator.h>
#include <google/protobuf/compiler/cpp/cpp_generator.h>
#include <google/protobuf/compiler/csharp/csharp_generator.h>
#include <google/protobuf/compiler/csharp/csharp_names.h>
#include <google/protobuf/compiler/php/php_generator.h>
#include <google/protobuf/compiler/java/java_names.h>
#include <google/protobuf/compiler/java/java_generator.h>
#include <google/protobuf/compiler/java/java_kotlin_generator.h>
#include <google/protobuf/compiler/js/js_generator.h>
#include <google/protobuf/compiler/js/well_known_types_embed.h>
#include <google/protobuf/compiler/ruby/ruby_generator.h>
```

<br>


---
---
Conan **1.47.0**. JFrog LTD. [https://conan.io](https://conan.io). Autogenerated 2022-04-26 11:21:29.