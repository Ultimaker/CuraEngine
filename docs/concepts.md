Overview
========

Storing Slice Data
----
Intermediate results obtained during slicing, such as the polygon shapes of a layer, are stored in the `SliceDataStorage` class and its child classes. This data is kept as short as possible, in order to reduce the memory footprint during slicing. Only the data of the previous stage in slicing is available. See the [pipeline](pipeline.md) page for details on these stages.

Fixed-point Coordinates
----
Coordinates are stored in 64-bit integers, representing the number of microns away from the coordinate origin. For instance, a difference of 1000 coordinate points represents a distance of 1mm.

This coordinate system is chosen because integers are less subject to unexpected rounding errors due to the changing resolution as you get further away from 0. Also, the library that CuraEngine uses for polygon optimisation works on 64-bit integers. Using microns gives high enough resolution and doesn't limit the size in which calculations can be made much.

Some care must be taken not to overflow these integers. Especially calculations involving Euclidean distance and area need to be careful of this, since they need to square the coordinates.

Parts
-----
Cura works with complex polygons that can have multiple contours. Some of these contours overlap the area covered by another contour. A *part* is a set of contours that does not overlap another set of contours.

For example, slicing a cube will result in one square cross-section for each layer. Such a layer consists of one part. However slicing a table will result in some layers containing multiple contours, one for each leg of the table. There will then be 4 parts on that layer.

Most operations in CuraEngine run on parts, since the application can then know that travel moves can safely be made within that part without needing to cross any walls. This minimises the amount of travel outside of the model.

Note that each part may still contain multiple contours if these polygons overlap. The part may be hollow.