


# arcus/latest@ultimaker/stable

---

Arcus is one of our own repositories and can be found: https://github.com/Ultimaker/libArcus For each commit to master
a new Conan package is created and uploaded to our JFrog Artifactory server. Such that it can be directly consumed
by CuraEngine.

<br>

## arcus/latest@ultimaker/stable dependencies

* [protobuf/3.17.1](https://conan.io/center/protobuf)
* [zlib/1.2.12](https://conan.io/center/zlib)

<br>

## Using the arcus Conan Package

<br>

Conan integrates with different build systems. You can declare which build system you want your project to use setting in the **[generators]** section of the [conanfile.txt](https://docs.conan.io/en/latest/reference/conanfile_txt.html#generators) or using the **generators** attribute in the [conanfile.py](https://docs.conan.io/en/latest/reference/conanfile/attributes.html#generators). Here, there is some basic information you can use to integrate **arcus** in your own project. For more detailed information, please [check the Conan documentation](https://docs.conan.io/en/latest/getting_started.html).



<br>

## Using arcus with CMake

<br>

### [Conan CMake generators](https://docs.conan.io/en/latest/reference/conanfile/tools/cmake.html)

<br>

* [CMakeDeps](https://docs.conan.io/en/latest/reference/conanfile/tools/cmake/cmakedeps.html): generates information about where the **arcus** library and its dependencies  ( [protobuf](https://conan.io/center/protobuf),  [zlib](https://conan.io/center/zlib)) are installed together with other information like version, flags, and directory data or configuration. CMake will use this files when you invoke ``find_package()`` in your *CMakeLists.txt*.

* [CMakeToolchain](https://docs.conan.io/en/latest/reference/conanfile/tools/cmake/cmaketoolchain.html): generates a CMake toolchain file the you can later invoke with CMake in the command line using `-DCMAKE_TOOLCHAIN_FILE=conantoolchain.cmake`.

Declare these generators in your **conanfile.txt** along with your **arcus** dependency like:

```ini
[requires]
arcus/latest6@ultimaker/stable

[generators]
CMakeDeps
CMakeToolchain
```

<br>

To use **arcus** in a simple CMake project with this structure:

```shell
.
|-- CMakeLists.txt
|-- conanfile.txt
`-- src
    `-- main..cpp
```

<br>

Your **CMakeLists.txt** could look similar to this, using the global **arcus::arcus** CMake's target:

```cmake
cmake_minimum_required(VERSION 3.15)
project(arcus_project CXX)

find_package(arcus)

add_executable(${PROJECT_NAME} src/main.cpp)

# Use the global target
target_link_libraries(${PROJECT_NAME} arcus::arcus)
```

<br>

To install **arcus/latest@ultimaker/stable**, its dependencies and build your project, you just have to do:

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



As the arcus Conan package defines components you can link only that desired part of the library in your project. For example, linking only with the arcus **libarcus** component, through the **arcus::libarcus** target.

```cmake
...
# Link just to arcus libarcus component
target_link_libraries(${PROJECT_NAME} arcus::libarcus)
```

<br>

To check all the available components for **arcus** Conan package, please check the dedicated section at the end of this document.






<br>

## Using arcus with Visual Studio

<br>

### [Visual Studio Conan generators](https://docs.conan.io/en/latest/reference/conanfile/tools/microsoft.html)

<br>

* [MSBuildDeps](https://docs.conan.io/en/latest/reference/conanfile/tools/microsoft.html#msbuilddeps): generates the **conandeps.props** properties file with information about where the **arcus** library and its dependencies  ( [protobuf](https://conan.io/center/protobuf),  [zlib](https://conan.io/center/zlib)) are installed together with other information like version, flags, and directory data or configuration.

* [MSBuildToolchain](https://docs.conan.io/en/latest/reference/conanfile/tools/microsoft.html#msbuildtoolchain): Generates the **conantoolchain.props** properties file with the current package configuration, settings, and options.

Declare these generators in your **conanfile.txt** along with your **arcus** dependency like:

```ini
[requires]
arcus/latest@ultimaker/stable

[generators]
MSBuildDeps
MSBuildToolchain
```

<br>

Please, [check the Conan documentation](https://docs.conan.io/en/latest/reference/conanfile/tools/microsoft.html) for more detailed information on how to add these properties files to your Visual Studio projects.



<br>

## Using arcus with Autotools and pkg-config

<br>

### [Autotools Conan generators](https://docs.conan.io/en/latest/reference/conanfile/tools/gnu.html)

<br>

* [AutotoolsToolchain](https://docs.conan.io/en/latest/reference/conanfile/tools/gnu/autotoolstoolchain.html): generates the **conanautotoolstoolchain.sh/bat** script translating information from the current package configuration, settings, and options setting some enviroment variables for Autotools like: ``CPPFLAGS``, ``CXXFLAGS``, ``CFLAGS`` and ``LDFLAGS``. It will also generate a ``deactivate_conanautotoolstoolchain.sh/bat`` so you can restore your environment.

* [AutotoolsDeps](https://docs.conan.io/en/latest/reference/conanfile/tools/gnu/autotoolsdeps.html): generates the **conanautotoolsdeps.sh/bat** script with information about where the **arcus** library and its dependencies  ( [protobuf](https://conan.io/center/protobuf),  [zlib](https://conan.io/center/zlib)) are installed together with other information like version, flags, and directory data or configuration. This is done setting some enviroment variables for Autotools like: ``LIBS``, ``CPPFLAGS``,``CXXFLAGS``, ``CFLAGS`` and ``LDFLAGS``.  It will also generate a ``deactivate_conanautotoolsdeps.sh/bat`` so you can restore your environment.

Declare these generators in your **conanfile.txt** along with your **arcus** dependency like:

```ini
[requires]
arcus/latest@ultimaker/stable

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

* [PkgConfigDeps](https://docs.conan.io/en/latest/reference/conanfile/tools/gnu/pkgconfigdeps.html): generates the **arcus.pc** file (and the ones corresponding to **arcus** dependencies) with information about the dependencies that can be later used by the **pkg-config** tool pkg-config to collect data about the libraries Conan installed.

<br>

You can use this generator instead of the **AutotoolsDeps** one:

```ini
[requires]
arcus/latest@ultimaker/stable

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

# You will have to set the PKG_CONFIG_PATH to where Conan created the arcus.pc file
$ export CPPFLAGS="$CPPFLAGS $(pkg-config --cflags arcus)"
$ export LIBS="$LIBS $(pkg-config --libs-only-l arcus)"
$ export LDFLAGS="$LDFLAGS $(pkg-config --libs-only-L --libs-only-other arcus)"

$ ./configure
$ make

# restore the environment after the build is completed
$ source deactivate_conanautotoolstoolchain.sh
```



<br>

As the arcus Conan package defines components you can use them to link only that desired part of the library in your project.  To check all the available components for **arcus** Conan package, and the corresponding *.pc* files names, please check the dedicated section at the end of this document.


<br>

## Other build systems

<br>

Please, [check the Conan documentation](https://docs.conan.io/en/latest/reference/conanfile/tools.html) for other integrations besides the ones listed in this document.





<br>

## Declared components for arcus

<br>

These are all the declared components for the **arcus** Conan package:
* Component **libarcus**:
  * CMake target name: ``arcus::libarcus``
  * pkg-config *.pc* file: **arcus-libarcus.pc**
  * Requires other components: **protobuf::protobuf**
  * Links to libraries: **Arcus**
  * Systems libs: **pthread**
  * Preprocessor definitions: ``ARCUS``
* Component **pyarcus**:
  * CMake target name: ``arcus::pyarcus``
  * pkg-config *.pc* file: **arcus-pyarcus.pc**
  * Requires other components: **libarcus**, **protobuf::protobuf**
  * Systems libs: **Python3.10**, **pthread**


<br>

## Exposed header files for arcus

<br>

```cpp
```

<br>


---
---
Conan **1.47.0**. JFrog LTD. [https://conan.io](https://conan.io). Autogenerated 2022-04-26 11:21:29.