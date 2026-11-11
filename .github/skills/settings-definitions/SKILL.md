---
name: settings-definitions
description: Find and interpret Cura settings definitions when adding, changing or reviewing CuraEngine settings reads, units, limits, defaults or settings ownership.
---

# Cura settings definitions

Use this skill when working on settings in CuraEngine, especially when their meaning, units, limits, dependencies or settings scope is unclear.

## Authoritative references

Refer to the skill declared in Cura for a global understanding of what they mean:
* https://raw.githubusercontent.com/Ultimaker/Cura/refs/heads/main/.github/skills/settings-definitions/SKILL.md

## Settings ownership and movement context

Select the settings source using the definition's scope properties and the context of the command:

- Settings with `settable_per_mesh` enabled belong at **Mesh** level.
- Settings with `settable_per_extruder` enabled, but not mesh-specific, belong at **ExtruderTrain** level.
- Settings with `settable_per_meshgroup` enabled, but neither mesh- nor extruder-specific, belong at **MeshGroup** level.
- Settings with none of those scopes enabled belong at the most global **Scene** level.

Always retrieve settings from the lowest applicable context: the associated mesh when available, otherwise the appropriate extruder, mesh group or scene. The feature's extruder is not necessarily the currently selected extruder.

Retrieving a higher-scope setting from a lower-scope context is safe and allowed. `Settings::get` handles parent lookup; a value absent locally but present in its parent is normal. Do not add presence checks, fallback values or ownership validation for normal inheritance.

## Values and units

CuraEngine receives a flattened list of fully resolved values at mesh, extruder and global contexts. The engine does not need to interpret `value` versus `default_value`, `enabled`, expression resolution or `children`. Values and limits in the definitions are guidance for understanding a setting and checking consistency, not instructions to reproduce frontend processing.

- Check `description`, `type` and `unit` before retrieving or converting a value.
- Whenever applicable, use the specific engine types: `coord_t`, `Velocity`, `Acceleration`, `LayerIndex`, `AngleDegrees`, `AngleRadians`, `Ratio`, `Duration`, `Temperature`, etc.
- The same setting retrieved through different `Settings::get<T>` specializations can have different units. For example, `get<double>` returns a distance in millimeters, while `get<coord_t>` converts that distance to integer micrometers. Verify the specialization in `src/settings/Settings.cpp` instead of assuming a type change is unit-neutral.

For example, `machine_max_feedrate_e` is filament speed in **mm/s**, whereas `material_max_flowrate` is volumetric flow in **mm³/s**. They are not interchangeable, and a volumetric G-code flavor requires appropriate unit conversion.
