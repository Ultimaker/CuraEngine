---
name: cpp-base
description: Base C++ conventions for CuraEngine, shared by the cpp-coding and code-review skills. Not meant to be used on its own.
---

# CuraEngine C++ base conventions

These rules apply both when writing and when reviewing C++ code in CuraEngine. The `cpp-coding` and `code-review` skills refer to this skill.

## Types and files

* Newly introduced types should either be privately nested in a class, or declared in their own header file.
  * A header should contain only its type. Very closely related types are also allowed, like a list of the declared type.
  * Implementation should be in a `.cpp` file as much as possible. Template classes/methods are the exception, but their use is discouraged unless really needed. Trivial methods, e.g. getters and setters, may be declared in the header.
  * Files should be placed in a folder where they logically make sense. Files at the root are allowed only for global processing functions.
* All member variables of a class should be private.
* Make use of explicitly defined types as much as possible.

## Naming and readability

* All variables and functions should have explicit names.
* Prefer explicit types over `auto`; `auto` is acceptable when it is clearly more readable.
* Prefer `for` loops over `while` loops when possible and if readability is not significantly reduced.
* Add short comments in very complex pieces of code.
* Document complex functions; trivial ones with an explicit signature, e.g. getters, don't need it.
* When calling functions with non-explicit arguments, like booleans, declare them above as a `constexpr` or `const` variable with an explicit name.

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
