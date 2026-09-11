# Agent Operational & Onboarding Guide (AGENTS.md)

This document explains the main structure of the CuraEngine application.

As a dynamic assistant, you must adhere strictly to these principles to maintain codebase sanity and ensure future developers can build upon your work efficiently.


# Global architecture

## Application description

The repository contains the full code to build CuraEngine, a standalone executable that implements the slicing of a 3D model into a GCode that can be read by a 3D printer. The global structure is the following:

* Load the 3D mesh(es) and their associated settings
* Slice the meshes to get a list of 2D polygons
* For each layer, turn the polygons into a list of extrusion paths that will form the model
* For each layer, translate the extrusion paths into actual GCode, while applying a few last-time modifications
* Send the extrusion data (with metadata) to the front-end, and the final gcode alongside

Since the input meshes can have very different shapes, we try to handle all the possible cases and use safe code as much as possible. We also focus very much on efficiency, since some meshes can have a very large number of triangles, or be large in physical size, which means the amount of generated extrusions is huge.

## Development

### Codebase
The codebase is essentially C++. Some parts of it are quite old, and possibly written at a time where there were no strict rules. But every time we make changes, we try to upgrade it with modern standards. The one we use is C++20, so not all features of modern C++ are available to us, because we need to support old platforms that don't support modern compilers. However we try to leverage the modern features as much as possible, in order to simplify our code, make it more portable and faster.

The application will be built on both Linux, Windows and Mac platforms. So we have many specific cases here and there for each platform, and it is important that they all keep working.

### Testing
Some parts of the application have very exhaustive unit tests. However we don't always add new tests when adding or changing a feature. Mostly when this is really relevant.

### Package management
Dependencies of CuraEngine are handled using conan2. Most of the recipes are taken from the conan center, but some are custom recipes that we have created/forked. CuraEngine is also a package that is consumed by the global application, Cura, that contains a front-end which calls CuraEngine.

### Project tools
The project uses various external tools:

* CMake for building
* protobuf to generate messages for the front-end application

It also has a few unit testing and benchmarking sub-projects that are run periodically, so it is critical that they keep working.
