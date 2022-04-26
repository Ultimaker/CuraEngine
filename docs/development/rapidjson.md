


# rapidjson/1.1.0

---



<br>

## Using the rapidjson Conan Package

<br>

Conan integrates with different build systems. You can declare which build system you want your project to use setting in the **[generators]** section of the [conanfile.txt](https://docs.conan.io/en/latest/reference/conanfile_txt.html#generators) or using the **generators** attribute in the [conanfile.py](https://docs.conan.io/en/latest/reference/conanfile/attributes.html#generators). Here, there is some basic information you can use to integrate **rapidjson** in your own project. For more detailed information, please [check the Conan documentation](https://docs.conan.io/en/latest/getting_started.html).



<br>

## Using rapidjson with CMake

<br>

### [Conan CMake generators](https://docs.conan.io/en/latest/reference/conanfile/tools/cmake.html)

<br>

* [CMakeDeps](https://docs.conan.io/en/latest/reference/conanfile/tools/cmake/cmakedeps.html): generates information about where the **rapidjson** library and its dependencies  are installed together with other information like version, flags, and directory data or configuration. CMake will use this files when you invoke ``find_package()`` in your *CMakeLists.txt*.

* [CMakeToolchain](https://docs.conan.io/en/latest/reference/conanfile/tools/cmake/cmaketoolchain.html): generates a CMake toolchain file the you can later invoke with CMake in the command line using `-DCMAKE_TOOLCHAIN_FILE=conantoolchain.cmake`.

Declare these generators in your **conanfile.txt** along with your **rapidjson** dependency like:

```ini
[requires]
rapidjson/1.1.0

[generators]
CMakeDeps
CMakeToolchain
```

<br>

To use **rapidjson** in a simple CMake project with this structure:

```shell
.
|-- CMakeLists.txt
|-- conanfile.txt
`-- src
    `-- main..c
```

<br>

Your **CMakeLists.txt** could look similar to this, using the global **rapidjson::rapidjson** CMake's target:

```cmake
cmake_minimum_required(VERSION 3.15)
project(rapidjson_project C)

find_package(rapidjson)

add_executable(${PROJECT_NAME} src/main.c)

# Use the global target
target_link_libraries(${PROJECT_NAME} rapidjson::rapidjson)
```

<br>

To install **rapidjson/1.1.0**, its dependencies and build your project, you just have to do:

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

## Using rapidjson with Visual Studio

<br>

### [Visual Studio Conan generators](https://docs.conan.io/en/latest/reference/conanfile/tools/microsoft.html)

<br>

* [MSBuildDeps](https://docs.conan.io/en/latest/reference/conanfile/tools/microsoft.html#msbuilddeps): generates the **conandeps.props** properties file with information about where the **rapidjson** library and its dependencies  are installed together with other information like version, flags, and directory data or configuration.

* [MSBuildToolchain](https://docs.conan.io/en/latest/reference/conanfile/tools/microsoft.html#msbuildtoolchain): Generates the **conantoolchain.props** properties file with the current package configuration, settings, and options.

Declare these generators in your **conanfile.txt** along with your **rapidjson** dependency like:

```ini
[requires]
rapidjson/1.1.0

[generators]
MSBuildDeps
MSBuildToolchain
```

<br>

Please, [check the Conan documentation](https://docs.conan.io/en/latest/reference/conanfile/tools/microsoft.html) for more detailed information on how to add these properties files to your Visual Studio projects.



<br>

## Using rapidjson with Autotools and pkg-config

<br>

### [Autotools Conan generators](https://docs.conan.io/en/latest/reference/conanfile/tools/gnu.html)

<br>

* [AutotoolsToolchain](https://docs.conan.io/en/latest/reference/conanfile/tools/gnu/autotoolstoolchain.html): generates the **conanautotoolstoolchain.sh/bat** script translating information from the current package configuration, settings, and options setting some enviroment variables for Autotools like: ``CPPFLAGS``, ``CXXFLAGS``, ``CFLAGS`` and ``LDFLAGS``. It will also generate a ``deactivate_conanautotoolstoolchain.sh/bat`` so you can restore your environment.

* [AutotoolsDeps](https://docs.conan.io/en/latest/reference/conanfile/tools/gnu/autotoolsdeps.html): generates the **conanautotoolsdeps.sh/bat** script with information about where the **rapidjson** library and its dependencies  are installed together with other information like version, flags, and directory data or configuration. This is done setting some enviroment variables for Autotools like: ``LIBS``, ``CPPFLAGS``,``CXXFLAGS``, ``CFLAGS`` and ``LDFLAGS``.  It will also generate a ``deactivate_conanautotoolsdeps.sh/bat`` so you can restore your environment.

Declare these generators in your **conanfile.txt** along with your **rapidjson** dependency like:

```ini
[requires]
rapidjson/1.1.0

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

* [PkgConfigDeps](https://docs.conan.io/en/latest/reference/conanfile/tools/gnu/pkgconfigdeps.html): generates the **rapidjson.pc** file (and the ones corresponding to **rapidjson** dependencies) with information about the dependencies that can be later used by the **pkg-config** tool pkg-config to collect data about the libraries Conan installed.

<br>

You can use this generator instead of the **AutotoolsDeps** one:

```ini
[requires]
rapidjson/1.1.0

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

# You will have to set the PKG_CONFIG_PATH to where Conan created the rapidjson.pc file
$ export CPPFLAGS="$CPPFLAGS $(pkg-config --cflags rapidjson)"
$ export LIBS="$LIBS $(pkg-config --libs-only-l rapidjson)"
$ export LDFLAGS="$LDFLAGS $(pkg-config --libs-only-L --libs-only-other rapidjson)"

$ ./configure
$ make

# restore the environment after the build is completed
$ source deactivate_conanautotoolstoolchain.sh
```




<br>

## Other build systems

<br>

Please, [check the Conan documentation](https://docs.conan.io/en/latest/reference/conanfile/tools.html) for other integrations besides the ones listed in this document.






<br>

## Exposed header files for rapidjson

<br>

```cpp
#include <rapidjson/encodings.h>
#include <rapidjson/pointer.h>
#include <rapidjson/filewritestream.h>
#include <rapidjson/memorybuffer.h>
#include <rapidjson/reader.h>
#include <rapidjson/rapidjson.h>
#include <rapidjson/ostreamwrapper.h>
#include <rapidjson/stream.h>
#include <rapidjson/writer.h>
#include <rapidjson/schema.h>
#include <rapidjson/allocators.h>
#include <rapidjson/memorystream.h>
#include <rapidjson/document.h>
#include <rapidjson/fwd.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/istreamwrapper.h>
#include <rapidjson/encodedstream.h>
#include <rapidjson/prettywriter.h>
#include <rapidjson/filereadstream.h>
#include <rapidjson/internal/strtod.h>
#include <rapidjson/internal/regex.h>
#include <rapidjson/internal/ieee754.h>
#include <rapidjson/internal/biginteger.h>
#include <rapidjson/internal/pow10.h>
#include <rapidjson/internal/strfunc.h>
#include <rapidjson/internal/swap.h>
#include <rapidjson/internal/diyfp.h>
#include <rapidjson/internal/meta.h>
#include <rapidjson/internal/dtoa.h>
#include <rapidjson/internal/itoa.h>
#include <rapidjson/internal/stack.h>
#include <rapidjson/msinttypes/stdint.h>
#include <rapidjson/msinttypes/inttypes.h>
#include <rapidjson/error/en.h>
#include <rapidjson/error/error.h>
```

<br>


---
---
Conan **1.47.0**. JFrog LTD. [https://conan.io](https://conan.io). Autogenerated 2022-04-26 11:21:29.