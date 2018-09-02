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

LayerParts
----------
An important concept to grasp is the LayerParts. LayerParts are seperate parts inside a single layer. For example, if you have a cube. Then each layer has a single LayerPart. However, if you have a table, then the layers which build the legs have a LayerPart per leg, and thus there will be 4 LayerParts.
A LayerPart is a seperated area inside a single layer which does not touch any other LayerParts. Most operations run on LayerParts as it reduces the amount of data to process. During GCode generation handling each LayerPart as an own step makes sure you never travel between LayerParts and thus reducing the amount of external travel.
LayerParts are generated after the Slicer step.

To generate the LayerParts Clipper is used. A Clipper union with extended results gives a list of Polygons with holes in them. Each polygon is a LayerPart, and the holes are added to this LayerPart.