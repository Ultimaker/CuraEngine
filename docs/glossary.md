Glossary
========
This page explains some of the terms used throughout CuraEngine's code and documentation. Some terms are broadly used throughout the 3D printing industry. Some are specific to Cura.

3D Printing Terms
----

Term | Meaning
---- | ----
Coasting | Coasting replaces the last part of an extrusion path by a travel move. This way, excess pressure is released from the nozzle, thus decreasing the chance of material leaking from the nozzle while travelling.
Extruder train | The feeder, optional Bowden tube, heat zone, hot end and nozzle together.
Extrusion | Pushing material out of a nozzle. The opposite of retraction.
Feeder | The motor and gear box that pushes the filament towards and through the nozzle.
Heat zone | The zone where the filament is heating up while going towards the nozzle. At the beginning of the heat zone the filament is at room temperature. At the end it is at printing temperature.
Layer | 3D printing is done by laying down thin layers of material. These layers are two-dimensional shapes with a small thickness, which when stacked vertically form a 3D object.
Nozzle | The opening where the filament exits the extruder train.
Retraction | The action of running the material feeder backwards for a short amount of time to pull the material away from the nozzle. This prevents passive oozing/dripping of materials in between actual printed lines.
Slicing | Ambiguous! This could refer either to the entire process of generating g-code that would print a 3D scene, or to a small part of that process that just generates cross-sectional areas at certain heights.
Support | If a portion of the model hangs over air, a structural support may need to be added to help holding the material in place, prevent it from falling down before it's solidified.
Travel | A nozzle movement without extruding any filament.
Z seam | Also known as layer seam or Z scar. This refers to the marks that appear on the prints where the nozzle starts and ends its loop around the perimeter.

Cura-specific Terms
----

Term | Meaning
---- | ---
Combing | A movement that tries to avoid crossing walls in order to prevent scarring the surface.
Inset | An operation on polygons that produces a smaller polygon whose contour keeps a certain distance from the contour of the original polygon. The polygon may also be a larger polygon if the inset distance is negative. This is used for generating walls, concentric infill, brims, support and many more features.
Link | The result of the mechanism to detect if there is any overlap between lines. If there is a link, there is an overlap.
Part | Also known as an island. This is a closed, isolated polygon that has no overlap with other closed polygons in the layer.
Polygon | A 2D shape created by connecting a series of points with straight line segments.
Shell | The top and bottom layers and the walls combined.
Skin | The top and bottom of the printed part. This is generated using a separate filling pattern.
Tower | A strut to reinforce parts of the print that would otherwise not be supported because they have no area. Used to support corners and edges pointing downwards.
Wall | The contour of a layer. They are typically comprised of inner walls and outer walls. The terms "inner" and "outer" do not refer to the actual shape of the printed object, but rather indicate on which side of the wall they are. The outer walls form the surface touching the air (and support material) outside the model. The inner walls form the surface touching the infill and skin.
Wire printing | Also known as weaving or Neith. A non-layer-wise form of printing a net of strings to create the rough shape of your model.
Winding order | A term in geometry indicating the order of the points forming a polygon. In CuraEngine, if the points around the polygon are in clockwise order, the polygon is considered to be a hole in the layer. If they are in counter-clockwise order, they are considered to be a solid area.