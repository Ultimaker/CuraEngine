---
name: cpp-base
description: Base C++ conventions for CuraEngine, shared by the cpp-coding and code-review skills. Not meant to be used on its own.
---

# CuraEngine C++ base conventions

These rules apply both when writing and when reviewing C++ code in CuraEngine. The `cpp-coding` and `code-review` skills refer to this skill.

## General good practices

* Take inspiration from the C++ Core Guidelines available here: https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines
* Do not enforce those guidelines, but use them if there is no clear convention defined in our own rules set.
* Use forward-declarations in header files as much as possible.

## Types and files

* Newly introduced types should either be privately nested in a class, or declared in their own header file. That includes structures, classes and enumerations.
  * A header should contain only its type. Very closely related types are also allowed, like a list of the declared type.
  * Implementation should be in a `.cpp` file as much as possible. Template classes/methods are the exception, but their use is discouraged unless really needed. Trivial methods, e.g. getters and setters, may be declared in the header.
  * Files should be placed in a folder where they logically make sense. Files at the root are allowed only for global processing functions.
* All member variables of a class should be private.
* Make use of explicitly defined types as much as possible.
* Each structure, class and enumeration should have their own dedicated header and cpp file. Empty files don't make sense and create confusion. Aliases and trivial definitions can remain attached to their associated struct/class.
* All the headers and cpp files names should match exactly the name of the type, case-sensitively. For instance, the GCodeExport class should be in the GCodeExport.h header and its code in GCodeExport.cpp. When a header only defines a namespace without types, the header should have the name of the namespace, and all the functions declared under the namespace are to be declared together.
* Place all the files in their appropriate folder. When creating multiple types that belong together and don't fit in an existing folder, a new folder should be created.
* Creating types that are local and nested to a class/function is allowed and encouraged when it improves the readability.

## Naming and readability

* Types should be using `UpperCamelCase`, e.g. `GCodeExport`.
* Functions should be using `lowerCamelCase`, e.g. `processCalculation`
* Variables should be using `snake_case, e.g. `calculated_deviation`.
* Private variable should have a `_` postfix, e.g. `line_width_`.
* Namsepaces should be using `snake_case`, and should preferably contain a single word.
* All variables and functions should have explicit names, no abbreviations.
* Prefer explicit types over `auto`; `auto` is acceptable when it is clearly more readable.
* Prefer `for` loops over `while` loops when possible and if readability is not significantly reduced.
* Add short comments in very complex pieces of code.
* Document complex functions; trivial ones with an explicit signature, e.g. getters, don't need it.
* When calling functions with non-explicit arguments, like booleans, declare them above as a `constexpr` or `const` variable with an explicit name.
* When an additional function associated to a type is required, e.g. a hash or formatting method, it should also be declared along with the type.
* All the headers should contain the top-level namespace `curaengine`. Types placed in sub-folders should have a matching namespace, e.g. a type `utils/math/Linear.h` should utilize the namespace `curaengine::utils::math`.
* When using code inside external namespaces, never declare a global `using`. In order to maintain readibility, you may declare a global alias to a namespace, e.g. instead of using `utils::math::statistics::Linear my_linear;`, you can do `namespace ums = utils::math::statistics; ums::Linear my_linear;`
* When using internal namespaces, they should be declared with a `using` in top of cpp files, omitting the `::curaengine` prefix. In header files, we should keep using the namespace prefix for each declaration, but still omit the `::curaengine`. If that turns out to be very long, it is allowed to used acronym compression as described above.

## Parameters, constness and ownership

* Pass non-primitive parameters by reference, using a const reference for read-only inputs, unless there is a good reason not to.
* Declare variables and function parameters `const` when possible.
* Use smart pointers when memory management and pointers come together (or when a collection isn't stable during the lifetime of a pointer). Raw pointers are allowed to refer to existing data; keep manual memory management to a minimum.

## Error handling and concurrency

* In new code, avoid exception-based control flow; prefer error handling by return value, unless exceptions are mandatory (e.g. required by an external library/API).
* Most of the code is processed in parallel: avoid race conditions and make sure the code is entirely repeatable across consecutive executions.

## Lambdas

* Lambdas declared inside a function are allowed, but their body should not exceed 30 lines, and broad captures (`[=]`, `[&]`) are not allowed.

## Libraries and algorithms

* Use existing libraries as much as possible instead of re-implementing their functionality.
* If a container operation can be done more simply via ranges/views, prefer that.
