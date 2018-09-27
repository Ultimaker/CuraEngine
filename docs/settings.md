Settings
====
Settings are stored in the `Settings` instances in the scene objects. How settings are obtained and how this scene is constructed is explained in this document.

Obtaining Settings
----
Settings are stored in all objects of the scene: The scene itself, the mesh groups, the extruder trains and the meshes. They are stored in instances of the `Settings` class.

The `Settings` class is a mapping from keys to `Setting` instances. Each of the `Setting` instances contains all of the properties of a setting. Currently this entails the setting value, stored as a string, and the limitation to a certain extruder, stored as a pointer to that extruder. The setting value is stored as string here and can later be interpreted in multiple ways. For instance, a setting indicating some angle could be interpreted in some places in degrees, and in some places in radians. Upon requesting the value of a setting, one has to indicate using template overloads which type should be returned, like so:

    const Settings& settings = Application::getInstance().current_slice->scene.settings;
    AngleDegrees angle_in_degrees = settings.get<AngleDegrees>("support_angle"); //Between 0 and 360.
    AngleRadians angle_in_radians = settings.get<AngleRadians>("support_angle"); //Between 0 and tau.

Some care must be taken with certain types of settings though:

* The number of extruders should be the same as the global `machine_extruder_count` setting, but the number of extruders in the scene is seen as leading. Please use `Application::getInstance().current_slice->scene.extruders.size()` instead of the setting value.
* Similarly, the extruder number settings (such as `support_infill_extruder_nr`) should be obtained using the `ExtruderTrain&` overload and then getting the extruder number from the resulting train.

Inheritance
----
Each `Settings` instance has a parent, except the one in the scene. If a setting is not present in the current `Settings` instance, the setting is requested from its parent. The `Scene` instance contains all settings. The hierarchy of scene objects reflects the parent-child relations in the `Settings` instances:

* The scene is the root of the tree. This contains the global settings.
* The parent of the mesh groups are the scene. These contain the per-mesh-group settings.
* The parent of the extruders is the current mesh group. When processing the next mesh group, the parents of the extruder trains are swapped to the next mesh group. These extruders contain the per-extruder settings.
* The parent of the meshes are the extruders that these meshes are printed with. The meshes contain the per-mesh settings.

Whether the setting is present or not depends on whether the front-end or the command line has sent it.

Limiting to extruder
----
If a print feature is set to be printed with a certain extruder, the extruder for some settings will be limited. For instance, if the infill is set to be printed with the second extruder, then all of the infill settings should get evaluated with the second extruder. This is done by evaluating the limitation to extruder first and if present, taking the setting from that extruder. Then the limitation to an extruder is not evaluated ever again. The final algorithm to evaluate a setting then looks like this:

1. If the current `Settings` instance has a value for the setting, return it. Otherwise...
2. If the setting is limited to a certain extruder, obtain it from that extruder. From then on, don't ever limit to an extruder any more (skip step 2). Otherwise...
3. If the current `Settings` instance has a parent, obtain the setting from that parent. Otherwise...
4. An error is raised. The setting doesn't exist any more!