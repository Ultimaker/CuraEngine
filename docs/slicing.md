Slicing
====
This document explains how CuraEngine creates slices (cross sections) of a 3D mesh.

"Slicing" is a confusing term since Cura is considered a "slicer" and it calls the process of transforming a 3D mesh into g-code the "slicing" process. This document is about a more technically correct definition of "slicing": The process of creating cross sections of a 3D mesh at certain heights.

Determining layer heights
----
Before creating cross sections of a 3D mesh, CuraEngine must first determine at what heights to create these cross sections.

Each layer is considered to have a certain span across the Z axis. For example, the first layer will have a span of 0 to 0.27mm, the second layer from 0.27mm to 0.37mm, the third layer from 0.37mm to 0.47mm, etc. The cross section of each layer will be taken through the _middle_ of each layer's span, by default. For the initial layer in this example, it would slice at a height of 0.135mm. The layer will printed from the height of the _top_ of the layer though. It would put a command to move to `Z0.27` before printing that layer.

Normally, the first layer has a separate layer height, the Initial Layer Height. The rest of the layers use the normal Layer Height setting.

![Layer Heights](assets/layer_heights.svg)

Alternatively, with Adaptive Layer Heights, the Z coordinates of the cross sections is determined based on the shape of the model. If Slicing Tolerance is set to Inclusive or Exclusive it will slice on the borders of layers instead of the middle.

Triangles to Lines
----
When the height of the cross section is determined, all triangles are intersected with the planes at every layer height, producing lines where they intersect.

![Triangle Line Intersection](assets/slice_triangle.svg)

For performance reasons, we iterate first over all triangles in the mesh. For each of these triangles, we then iterate over the layers that this triangle intersects with and for each of these layers we produce a line segment at the intersection. This ordering of loops is unintuitive, but more efficient. It is easy to determine for a triangle which layers it intersects with by just looking at the Z coordinates of its 3 vertices, so we don't need to check every layer for every triangle but just the ones that would produce intersections.

To find the intersection of a plane and a triangle, we simply interpolate all three line segments of the triangle. At least two of these interpolations should span the plane. We take the two coordinates where the interpolations have the same Z coordinate as the plane and those will become the two endpoints of the line segment.