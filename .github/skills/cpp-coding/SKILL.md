---
name: cpp-coding
description: Apply CuraEngine C++ conventions when adding or changing classes, structs, methods and function signatures.
---

# CuraEngine C++ coding conventions

Use this skill when implementing or changing C++ code in CuraEngine.

- Place method definitions in the corresponding `.cpp` file, not inline in the class declaration. Template definitions that must be visible at instantiation remain in headers.
- Leave an empty line between method declarations.
- Give each class or struct member variable a brief description of what it contains, including units when relevant.
- Pass values that are not basic types by reference; use a const reference for read-only inputs.
- Prefer explicit types over `auto` whenever possible.
- Use the specific engine types for physical quantities and settings reads. Consult the `settings-definitions` skill for settings sources, inheritance and retrieval units.

Apply these conventions to changed code without rewriting unrelated existing code.
