---
name: cpp-coding
description: Apply CuraEngine C++ conventions when adding or changing classes, structs, methods and function signatures.
---

# CuraEngine C++ coding conventions

Use this skill when implementing or changing C++ code in CuraEngine.

First apply the shared conventions from the `cpp-base` skill, then the following:

- Place method definitions
- Leave an empty line between method declarations.
- Give each class or struct member variable a brief description of what it contains, including units when relevant.
- Use the specific engine types for physical quantities and settings reads. Consult the `settings-definitions` skill for settings sources, inheritance and retrieval units.

Apply these conventions to changed code without rewriting unrelated existing code.
