Pipeline
====
This document serves as an overview of how the slicing process works from front to back.

There are 5 main stages involved in slicing an object. Not all of them are executed strictly in sequence though. To reduce memory overhead and improve multi-threaded performance, CuraEngine executes some of these steps in a producer-consumer pattern: One thread produces data in an earlier stage that is consumed by another thread computing a later stage at the same time.

[Slicing](slicing.md)
----
The first step in the pipeline is the slicing stage. This converts a 3D mesh into 2D layers. First it determines the heights at which to produce cross sections. Then it creates cross sections of every triangle at those heights, which will normally be a line. Then it stitches these lines together to form polygons wherever possible.

[Generating Areas](generating_areas.md)
----
When given a set of layers, this stage divides each layer up into the areas that will be filled with the types of things we're going to print. What part is going to become wall? What part will be infill? Where will we place support?

[Generating Paths](generating_paths.md)
----
This stage is the most complex part of CuraEngine. Here the areas that were generated in the previous step will be actually filled with lines. The order in which we print the lines is determined here also. The output of this stage is a set of `LayerPlan`s, which contain the movement commands that the printer will eventually execute and in what order they will be executed.

[Inserts](inserts.md)
----
After the plan is created we can still make modifications to the plan to some extent. We have to insert commands to pre-heat and pre-cool the nozzles before an extruder switch. In order to predict how far ahead the nozzles must start heating and cooling, time estimates are generated for each path and the temperature change commands are inserted such that the time we need to heat or cool will be covered in the upcoming moves.

[G-code](gcode_export.md)
----
Finally, the plans that we've generated, including the temperature inserts, are translated from CuraEngine's internal representation to g-code.