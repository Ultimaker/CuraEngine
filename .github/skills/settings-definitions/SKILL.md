---
name: settings-definitions
description: Find and interpret Cura settings definitions when adding, changing or reviewing CuraEngine settings reads, units, limits, defaults or settings ownership.
---

# Cura settings definitions

Use this skill when working on settings in CuraEngine, especially when their meaning, units, limits, dependencies or settings scope is unclear.

## Authoritative references

Settings definitions are maintained in the public **Ultimaker/Cura** repository, not CuraEngine:

- Base printer settings: https://github.com/Ultimaker/Cura/blob/main/resources/definitions/fdmprinter.def.json
- Raw base definitions: https://raw.githubusercontent.com/Ultimaker/Cura/main/resources/definitions/fdmprinter.def.json
- Other printer and extruder definitions: https://github.com/Ultimaker/Cura/tree/main/resources/definitions
- Setting Properties documentation: https://github.com/Ultimaker/Cura/wiki/Setting-Properties
- Raw wiki fallback: https://raw.githubusercontent.com/wiki/Ultimaker/Cura/Setting-Properties.md

Read the Setting Properties documentation alongside the definitions. Use the Cura revision relevant to the task; if using `main`, state that explicitly rather than assuming it matches a particular CuraEngine release. If access fails, notify the user and ask for the relevant definitions instead of guessing.

## Settings ownership and movement context

Select the settings source using the definition's scope properties and the context of the command:

- Settings with `settable_per_mesh` enabled belong at **Mesh** level.
- Settings with `settable_per_extruder` enabled, but not mesh-specific, belong at **ExtruderTrain** level.
- Settings with `settable_per_meshgroup` enabled, but neither mesh- nor extruder-specific, belong at **MeshGroup** level.
- Settings with none of those scopes enabled belong at the most global **Scene** level.

Use the mesh associated with the path and the appropriate extruder for the feature being emitted. Do not assume that a feature's extruder is always the currently selected extruder; inspect `limit_to_extruder`. Preserve existing settings inheritance and resolution instead of duplicating or relocating settings without checking their scope. When a move has no associated mesh, establish the applicable settings context explicitly.

The wiki describes scope flags as permissions to override settings and notes that `settable_per_meshgroup` is currently unused by Cura. The ownership mapping above is CuraEngine contributor guidance; it does not imply that every value is physically present in that object's settings map. Inspect the effective properties and inherited definitions; do not assume an omitted property is explicitly false.

## Interpreting properties

- Check `description`, `type` and `unit` before choosing a C++ type or converting units.
- Distinguish `value` expressions from `default_value` literals. Cura resolves expressions; do not add a Python-expression evaluator to CuraEngine.
- Inspect `resolve` and `limit_to_extruder` when determining effective values.
- `enabled` controls whether a setting can be shown to the user; it is not by itself an engine runtime enable switch.
- `minimum_value` and `maximum_value` describe frontend validation, while the corresponding warning properties describe warnings. Neither substitutes for required enforcement in generated output.
- Follow nested `children` and relevant inherited printer/extruder definitions to find feature-specific settings and overrides.

For example, `machine_max_feedrate_e` is filament speed in **mm/s**, whereas `material_max_flowrate` is volumetric flow in **mm³/s**. They are not interchangeable, and a volumetric G-code flavor requires appropriate unit conversion.

## Check the engine lookup

Before changing a settings read, inspect `Settings::get` in `src/settings/Settings.cpp`: local values are checked first, then extruder limiting, then parent settings. Compare existing callers and path configurations with the definition's scope. If definitions, caller context or the requested behavior leave a decision unresolved, ask rather than inventing a fallback.
